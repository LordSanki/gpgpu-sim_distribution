#include "capri.h"
#include <limits>
#include <iostream>

using namespace std;
using namespace CAPRI;

Capri *Capri::m_obj;

Capri* Capri::getCapriObj()
{
  if(m_obj == NULL)
    m_obj = new Capri();
  return m_obj;
}
void Capri::releaseCapriObj()
{
  if(m_obj){
    delete m_obj;
    m_obj = NULL;
  }
}

static double pdom_util_func(Capri::Measurements &m)
{
  return m.m_pdom_util;
}
static double tbc_util_func(Capri::Measurements &m)
{
  return m.m_tbc_util;
}
static double capri_util_func(Capri::Measurements &m)
{
  return m.m_capri_util;
}

Capri::Capri()
{
  /*
  m_mispredictions = 0;
  m_non_divergent_inst_count = 0;
  m_adq_branches = 0;
  m_inadq_branches = 0;
  m_total_inst_count = 0;
  m_tbc_util = 0;
  m_capri_util = 0;
  m_pdom_util = 0;
  */
  m_currMeasurePtr = NULL;
}

Capri::~Capri()
{
}

void Capri::store(TBID tbid, int wid, int opcode, long pc, BitMask mask)
{
    Trace::iterator it = m_trace.find(tbid);
    if(it == m_trace.end()){
      it = m_trace.insert( pair<TBID,ThreadBlock>(tbid, ThreadBlock()) ).first;
    }
    ThreadBlock &tblock = it->second;
    while((int)tblock.size() <= wid){
      tblock.push_back(Warp());
    }
    Instruction ins;
    ins.op = (OpCodes)opcode;
    ins.pc = pc;
    ins.mask = mask;
    tblock[wid].push_back(ins);
}

void Capri::process()
{
  for(Trace::iterator itb = m_trace.begin(); itb != m_trace.end(); itb++){
    ThreadBlock &tblock = itb->second;
    int wid = get_min_pc_wid(tblock);
    m_stack = SimdStack();
    m_measurements[itb->first] = Measurements();
    m_currMeasurePtr = &m_measurements[itb->first];
    while(wid != -1){
      m_currMeasurePtr->m_total_inst_count++;
      Instruction curr = tblock[wid].front();

      // updating top of stack
      if(m_stack.empty() == false){
        // found a reconvergence point. Pop the top of stack
        // and update global counter
        SimdStackElem elem = m_stack.top();
        if(curr.mask == m_stack.top().mask)
        {
          m_currMeasurePtr->m_pdom_util += (m_stack.top().pdom.factor * m_stack.top().pdom.count);
          m_currMeasurePtr->m_tbc_util += (m_stack.top().tbc.factor * m_stack.top().tbc.count);
          m_currMeasurePtr->m_capri_util += (m_stack.top().capri.factor * m_stack.top().capri.count);
          m_currMeasurePtr->m_pdom_count += ( m_stack.top().pdom.count);
          m_currMeasurePtr->m_tbc_count += ( m_stack.top().tbc.count);
          m_currMeasurePtr->m_capri_count += ( m_stack.top().capri.count);
          m_stack.pop();
          if(m_stack.empty()){
            m_currMeasurePtr->m_non_divergent_inst_count++;
          }
          else{
            m_stack.top().capri.count++;
            m_stack.top().tbc.count++;
            m_stack.top().pdom.count++;
          }
        }
        // not a reconv point. increment counter
        else{
          m_stack.top().capri.count++;
          m_stack.top().tbc.count++;
          m_stack.top().pdom.count++;
        }
      }
      else{
        m_currMeasurePtr->m_non_divergent_inst_count++;
      }
      // executing WCU and CAPT if BRANCH_OP encountered
      if(curr.op == BRANCH_OP){
        m_stack.push(SimdStackElem(curr.mask));
        check_adequacy(curr, tblock);
      }
      else{
        for(int w = 0; w < (int)tblock.size(); w++){
          if(tblock[w].empty()) continue;
          if(tblock[w].front().pc == curr.pc)
            tblock[w].pop_front();
        }
      }
      // finding next warp id
      wid = get_min_pc_wid(tblock);
    }
  }
}

void Capri::check_adequacy( Instruction &curr, ThreadBlock &tblock)
{
  int wcount=0;
  int mask_count[32] = {0};
  for(int w=0; w<(int)tblock.size(); w++){
    if(tblock[w].front().pc == curr.pc){
      tblock[w].pop_front();
      for(int b=0; b<32; b++){
        mask_count[b] += tblock[w].front().mask[b];
      }
      wcount++;
    }
  }

  int taken_count = mask_count[0];
  int ntaken_count = mask_count[0];
  for(int m=0; m<32; m++){
    if(mask_count[m] > taken_count)
      taken_count = mask_count[m];
    if(mask_count[m] < ntaken_count)
      ntaken_count = mask_count[m];
  }

  ntaken_count = wcount - ntaken_count;

  bool non_diverging_branch = true;
  for(int b=0; b<32; b++){
    if(mask_count[b] != wcount)
      non_diverging_branch = false;
  }
  double compact_factor = taken_count+ntaken_count;
  if(compact_factor > 0.0)
    compact_factor = wcount/compact_factor;
  else
    compact_factor = 1;
  // if capt predicts adq
  if(m_capt(curr.pc) ){
    // check if it was actually adq
    if(taken_count < wcount || ntaken_count < wcount){
      // inrement counter
      if(non_diverging_branch == false)
        m_currMeasurePtr->m_adq_branches++;
      // set factor
      m_stack.top().capri.factor = compact_factor;
      m_capt(curr.pc, true);
    }
    else{
      m_currMeasurePtr->m_inadq_branches++;
      // misprediction by capt
      m_currMeasurePtr->m_mispredictions++;
      // set factor
      m_stack.top().capri.factor = 0.5;
      m_capt(curr.pc, false);
    }
  }
  // capt predicts inadq
  else{
    m_stack.top().capri.factor = 0.5;
    // if branch is adq
    if(taken_count < wcount || ntaken_count < wcount){
      // misprediction by capt
      m_currMeasurePtr->m_mispredictions++;
      m_capt(curr.pc, true);
      m_currMeasurePtr->m_adq_branches++;
    }
    else{
      m_currMeasurePtr->m_inadq_branches++;
      m_capt(curr.pc, false);
    }
  }

  if(non_diverging_branch){
    m_stack.top().capri.factor = 1.0;
    m_stack.top().pdom.factor = 1.0;
  }
  else{
    m_stack.top().pdom.factor = 0.5;
  }
  m_stack.top().tbc.factor = compact_factor;
}

int Capri::get_min_pc_wid(ThreadBlock &tblock)
{
  int wid = -1;
  long min_pc = numeric_limits<long>::max();
  for(int w=tblock.size()-1; w >= 0; w--)
  {
    if(tblock[w].empty()) continue;
    if(tblock[w].front().pc < min_pc){
      wid = w;
      min_pc = tblock[w].front().pc;
    }
  }
  return wid;
}

void Capri::print_result()
{
  long count = 0;
  long double capri_pred_rate=0, pdom_pred_rate=0, tbc_pred_rate=0;
  for(TBMeasurements::iterator it = m_measurements.begin();
      it!= m_measurements.end(); it++){
    long double total_branches = it->second.m_adq_branches + it->second.m_inadq_branches;
    long double capri_rate = (total_branches - it->second.m_mispredictions);
    long double pdom_rate = it->second.m_inadq_branches;
    long double tbc_rate = it->second.m_adq_branches;
    if(total_branches > 0.0){
      capri_rate /= total_branches;
      pdom_rate /= total_branches;
      tbc_rate /= total_branches;
      capri_pred_rate += capri_rate;
      pdom_pred_rate += pdom_rate;
      tbc_pred_rate += tbc_rate;
      count++;
    }
  }
  if(count > 0){
    capri_pred_rate /= count;
    tbc_pred_rate /= count;
    pdom_pred_rate /= count;
  }
  else{
    capri_pred_rate =1;
    tbc_pred_rate /= 1;
    pdom_pred_rate /= 1;
  }
  cout<<"\n\n====================================================\n\n\n";
  print_simd_util(pdom_util_func, "PDOM");
  print_simd_util(tbc_util_func, "TBC");
  print_simd_util(capri_util_func, "CAPRI");
  cout<<"Avg PDOM Prediction Rate:\t"<<pdom_pred_rate<<"\n";
  cout<<"Avg TBC Prediction Rate:\t"<<tbc_pred_rate<<"\n";
  cout<<"Avg CAPRI Prediction Rate:\t"<<capri_pred_rate<<"\n";
  cout<<"\n\n====================================================\n\n";
}

void Capri::print_simd_util(Util_func func, const char * name)
{
  long double simd_util = 0;
  long count=0;
  for(TBMeasurements::iterator it = m_measurements.begin();
      it!= m_measurements.end(); it++)
  {
    long double util = func(it->second) + it->second.m_non_divergent_inst_count;
    //cout<<" Factor:\t"<<func(it->second)<<"\tNonD:\t"<<it->second.m_non_divergent_inst_count<<"\tTot:\t"<<it->second.m_total_inst_count<<"\tCont:\t"<<it->second.m_pdom_count<<"\n";
    if(it->second.m_total_inst_count > 0.0){
      simd_util += (util/it->second.m_total_inst_count);
      count++;
    }
  }
  if(count > 0)
    simd_util /= count;
  else
    simd_util = 1;
  cout<<"Avg "<<name<<" SIMD Utilization:\t"<<simd_util<<"\n";
}


