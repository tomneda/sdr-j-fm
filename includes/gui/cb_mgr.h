#pragma once

#include "qsettings.h"
#include <qobject.h>
#include <qcombobox.h>
#include <qstring.h>
#include <cstdint>
#include <vector>
#include <map>

// tomneda: compromise with a macro as I got problem porting the handler function pointer via the constructor of CbElem
#define CBELEM(sp_,cbid_,pqcb_,pdst_,phdl_)      auto sp_ = std::make_shared<CbElem>(cbid_,pqcb_); \
                                                 connect(pqcb_, &QComboBox::textActivated, pdst_, phdl_);
  

namespace NCbDef
{
  using TCbId   = uint32_t; // distinct ID for each combo box 
  using TItem   = uint32_t; // content of each combo box 
  using TDefSel = uint32_t; // bitpattern to find a certain element for default setting
  
  enum ECbId : TCbId
  {
    CBID_RDS,
    CBID_FMMODE,
    CBID_AUDCH,
    CBID_DEEMP
  };
  
  enum EItemFmMode : TItem 
  {
    FMMODE_MONO,
    FMMODE_STEREO,
    FMMODE_STEREO_PANO,
    FMMODE_STEREO_AUTO
  };
  
  enum EItemDeEmp : TItem 
  {
    DEEMP_OFF =  0,
    DEEMP_25  = 25,
    DEEMP_50  = 50,
    DEEMP_75  = 75
  };
  
  enum EDefSel : TDefSel
  {
    DEFSEL_NONE = 0x0000,
    DEFSEL_EUR  = 0x0001, 
    DEFSEL_USA  = 0x0002,
    DEFSEL_ALL  = DEFSEL_USA | DEFSEL_EUR
  };
};


class CbElem //: public QObject
{
  //Q_OBJECT

public:
  CbElem(NCbDef::TCbId iCbId, QComboBox * const ipComboBox /*, QObject * const iReceiver*/ /*, const TFunc & iMethod*/);
  CbElem & operator=(const CbElem &) = default; 	
  CbElem(const CbElem & other) = default;
  ~CbElem() = default;
  
  struct SItem
  {
    NCbDef::TItem   Item;
    NCbDef::TDefSel DefSel;
    QString         EntryName;
  };
  
  void addItem(const NCbDef::TItem iItem, const NCbDef::TDefSel iDefSel, const QString & iEntryName);
  void set_current_selected_item_by_name(const QString & iItemName);
  NCbDef::TItem get_current_selected_item_id();
  NCbDef::TItem get_item_id_of_def_sel(const NCbDef::TDefSel iDefSel) const;
  const QString & get_item_name_of_def_sel(const NCbDef::TDefSel iDefSel) const;
  
  NCbDef::TCbId get_cb_id() const { return mCbId; }
  QComboBox * get_cb_box_ptr() const { return mpComboBox; }
  
private:
  const SItem & get_item_of_def_sel(const NCbDef::TDefSel iDefSel) const;
  
  NCbDef::TCbId      mCbId;
  QComboBox        * mpComboBox;
  std::vector<SItem> mItems;
};


using TSPCbElem = std::shared_ptr<CbElem>;

class CbElemColl
{
public:
  CbElemColl() = default;
  //~CbElemColl() = default;
  
  void set_setting_handler(QSettings *const ipS) { mpQSetting = ipS; }
  void read_cb_from_setting(const NCbDef::TDefSel iUseDefSel, const bool iForceDefaults = false);
  void write_setting_from_cb();
  
  void store_cb_elem(const TSPCbElem & ipCbElem, const QString & iGrpName, const QString & iCbName);
  TSPCbElem get_cb_elem_from_id(const NCbDef::TCbId iCbId);
  
private:
  struct SMapElem
  {
    TSPCbElem pCbElem;
    QString GrpName;
    QString CbName;
  };
  
  using TMap = std::map<NCbDef::TCbId, SMapElem>;
  TMap mCbElems;   
  QSettings *mpQSetting = nullptr;
  
  bool is_only_one_bit_set(const uint32_t iBitSet) const;
};


