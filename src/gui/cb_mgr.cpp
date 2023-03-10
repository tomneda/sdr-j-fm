#include "cb_mgr.h"


CbElem::CbElem(ECbId iCbId, const QString & iSettingName, QComboBox * const ipComboBox
       /*, QObject * const iReceiver*/ /*, const TFunc & iMethod*/) : 
  mCbId(iCbId), 
  mSettingName(iSettingName),
  mpComboBox(ipComboBox)
{
  //(void)iReceiver;
  //(void)iMethod;
  //connect(ipComboBox, &QComboBox::activated, iReceiver, iMethod);
  //qInfo("I am called!!!");
}


void CbElem::addItem(const TItem iItem, const EStartSetting iStartSetting, const QString & iEntryName)
{
  SItem item;
  item.Item = iItem;
  item.StartSetting = iStartSetting;
  item.EntryName = iEntryName;
  mItems.push_back(item);
  
  mpComboBox->addItem(iEntryName);
}


TItem CbElem::get_item_id()
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
  CbElem::ECbId cbId = ipCbElem->get_cb_id();
  Q_ASSERT(mCbElems.count(cbId) == 0);
  //mCbElems.insert(std::make_pair(cbId, ipCbElem));
  mCbElems[cbId] = ipCbElem;
}


TSPCbElem CbElemColl::operator[](const CbElem::ECbId iCbId)
{
  Q_ASSERT(mCbElems.count(iCbId) > 0);
  return mCbElems[iCbId];
}


void CbElemColl::read_cb_from_setting()
{
  Q_ASSERT(mpQSetting);
  //mpQSetting->beginGroup ("Main");

  for (auto const & [cbId, cbElem]  : mCbElems)
  {
    //cbElem:
  }
  
  //h = s->value("fmMode", fmModeSelector->currentText()).toString();

  //mpQSetting->endGroup();
}
