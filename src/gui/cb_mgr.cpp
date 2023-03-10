#include "cb_mgr.h"

using namespace NCbDef;

CbElem::CbElem(TCbId iCbId, const QString & iSettingName, QComboBox * const ipComboBox /*, QObject * const iReceiver*/ /*, const TFunc & iMethod*/) : 
  mCbId(iCbId), 
  mSettingName(iSettingName),
  mpComboBox(ipComboBox)
{
  //(void)iReceiver;
  //(void)iMethod;
  //connect(ipComboBox, &QComboBox::activated, iReceiver, iMethod);
  //qInfo("I am called!!!");
}


void CbElem::addItem(const TItem iItem, const TDefSel iDefSel, const QString & iEntryName)
{
  SItem item;
  item.Item = iItem;
  item.DefSel = iDefSel;
  item.EntryName = iEntryName;
  mItems.push_back(item);

  mpComboBox->addItem(iEntryName);
}


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

// -----------------------------------------------------------------------

void CbElemColl::store_cb_elem(const TSPCbElem & ipCbElem)
{
  TCbId cbId = ipCbElem->get_cb_id();
  Q_ASSERT(mCbElems.count(cbId) == 0);
  //mCbElems.insert(std::make_pair(cbId, ipCbElem));
  mCbElems[cbId] = ipCbElem;
}


TSPCbElem CbElemColl::get_cb_elem_from_id(const TCbId iCbId)
{
  Q_ASSERT(mCbElems.count(iCbId) > 0);
  return mCbElems[iCbId];
}


void CbElemColl::read_cb_from_setting()
{
  Q_ASSERT(mpQSetting);
  //mpQSetting->beginGroup ("Main");

  for (const auto & [cbId, pCbElem] : mCbElems)
  {
    Q_ASSERT(cbId == pCbElem->get_current_selected_item_id());
    const QString & itemName = pCbElem->get_setting_item_name();
    QComboBox * pCb = pCbElem->get_cb_box_ptr();
    QString h = mpQSetting->value(itemName, pCb->currentText()).toString();
    //h = s->value("fmMode", fmModeSelector->currentText()).toString();
  }
  

  //mpQSetting->endGroup();
}
