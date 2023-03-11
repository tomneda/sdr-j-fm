#include "cb_mgr.h"

using namespace NCbDef;

/******************************************************************************/
CbElem::CbElem(TCbId iCbId, QComboBox * const ipComboBox /*, QObject * const iReceiver*/ /*, const TFunc & iMethod*/) : 
  mCbId(iCbId), 
  mpComboBox(ipComboBox)
{
  //(void)iReceiver;
  //(void)iMethod;
  //connect(ipComboBox, &QComboBox::activated, iReceiver, iMethod);
  //qInfo("I am called!!!");
}

/******************************************************************************/
void CbElem::addItem(const TItem iItem, const TDefSel iDefSel, const QString & iEntryName)
{
  SItem item;
  item.Item = iItem;
  item.DefSel = iDefSel;
  item.EntryName = iEntryName;
  mItems.push_back(item);

  mpComboBox->addItem(iEntryName);
}

/******************************************************************************/
void CbElem::set_current_selected_item_by_name(const QString & iItemName)
{
  const int idx = mpComboBox->findText(iItemName);
  Q_ASSERT(idx >= 0);
  mpComboBox->setCurrentIndex(idx);
}

/******************************************************************************/
TItem CbElem::get_current_selected_item_id()
{
  const QString qs = mpComboBox->currentText();
  
  for (const auto & item : mItems)
  {
    if (qs == item.EntryName)
    {
      return item.Item;
    }
  }
  Q_ASSERT(0);
  
  return 0;
}

/******************************************************************************/
TItem CbElem::get_item_id_of_def_sel(const TDefSel iDefSel) const
{
  return get_item_of_def_sel(iDefSel).Item;
}

/******************************************************************************/
const QString & CbElem::get_item_name_of_def_sel(const TDefSel iDefSel) const
{
  return get_item_of_def_sel(iDefSel).EntryName;
}

/******************************************************************************/
const CbElem::SItem & CbElem::get_item_of_def_sel(const TDefSel iDefSel) const
{
  for (const auto & item : mItems)
  {
    if (item.DefSel & iDefSel) // suitable bit set?
    {
      return item;
    }
  }
  Q_ASSERT(0);
  
  return mItems[0]; // should never happen
}

/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
void CbElemColl::store_cb_elem(const TSPCbElem & ipCbElem, const QString & iGrpName, const QString & iCbName)
{
  TCbId cbId = ipCbElem->get_cb_id();
  Q_ASSERT(mCbElems.count(cbId) == 0);
  SMapElem me;
  me.pCbElem = ipCbElem;
  me.GrpName = iGrpName;
  me.CbName = iCbName;
  //mCbElems.insert(std::make_pair(cbId, me));
  mCbElems[cbId] = me;
}

/******************************************************************************/
TSPCbElem CbElemColl::get_cb_elem_from_id(const TCbId iCbId)
{
  Q_ASSERT(mCbElems.count(iCbId) > 0);
  return mCbElems[iCbId].pCbElem;
}

/******************************************************************************/
void CbElemColl::read_cb_from_setting(const TDefSel iUseDefSel, const bool iForceDefaults /*= false*/)
{
  Q_ASSERT(mpQSetting);
  Q_ASSERT(is_only_one_bit_set(iUseDefSel));
  bool anyDefaultSet = false;
  
  for (const auto & [cbId, cbElem] : mCbElems)
  {
    Q_ASSERT(cbId == cbElem.pCbElem->get_cb_id());
    
    const QString & defEntryName = cbElem.pCbElem->get_item_name_of_def_sel(iUseDefSel);
    const QString entryName = (iForceDefaults ? defEntryName : mpQSetting->value(cbElem.CbName, defEntryName).toString());
    
    mpQSetting->beginGroup (cbElem.GrpName);
    cbElem.pCbElem->set_current_selected_item_by_name(entryName);
    mpQSetting->endGroup();
    
    anyDefaultSet |= (entryName != defEntryName);
  }
  
  if (iForceDefaults || anyDefaultSet)
  {
    write_setting_from_cb();
  }
}

/******************************************************************************/
void CbElemColl::write_setting_from_cb()
{
  Q_ASSERT(mpQSetting);
    
  for (const auto & [cbId, cbElem] : mCbElems)
  {
    Q_ASSERT(cbId == cbElem.pCbElem->get_cb_id());
    QComboBox * pCb = cbElem.pCbElem->get_cb_box_ptr();

    mpQSetting->beginGroup (cbElem.GrpName);
    mpQSetting->setValue(cbElem.CbName, pCb->currentText());
    mpQSetting->endGroup();
  }
  
  mpQSetting->sync();
}

/******************************************************************************/
bool CbElemColl::is_only_one_bit_set(const uint32_t iBitSet) const 
{
  return (iBitSet && !(iBitSet & (iBitSet - 1)));
}

