#pragma once

#include "qsettings.h"
#include <qobject.h>
#include <qcombobox.h>
#include <qstring.h>
#include <cstdint>
#include <vector>
#include <map>

// tomneda: compromise with a macro as I got problem porting the handler function pointer via the constructor of CbElem
#define CBELEM(i_,a_,b_,c_,d_,e_)   auto cbe = std::make_shared<CbElem>(a_,b_,c_); \
                                    connect(c_, &QComboBox::textActivated, d_, e_);
  

using TItem = uint32_t;

enum EItemFmMode : TItem // insert to type EItem
{
  MONO,
  STEREO,
  STEREO_PANO,
  STEREO_AUTO
};

using EPrio = uint32_t;


class CbElem //: public QObject
{
  //Q_OBJECT

public:
  enum ECbId
  {
    CBID_RDS,
    CBID_FMMODE
  };

  enum EStartSetting
  {
    NONE = 0x0000,
    EU   = 0x0001,
    USA  = 0x0002,
    BOTH = USA | EU
  };
  
  //using TFunc = void (*)(const QString&);
  //template <typename TFunc>
  CbElem(ECbId iCbId, const QString & iSettingName, QComboBox * const ipComboBox
         /*, QObject * const iReceiver*/ /*, const TFunc & iMethod*/);
  CbElem & operator=(const CbElem &) = default; 	
  CbElem(const CbElem & other) = default;
  ~CbElem() = default;
  
  struct SItem
  {
    TItem         Item;
    EStartSetting StartSetting;
    QString       EntryName;
  };
  
  inline ECbId get_cb_id() const { return mCbId; }

  void addItem(const TItem iItem, const EStartSetting iStartSetting, const QString & iEntryName);
  TItem get_item_id();
  
private:
  ECbId mCbId;
  QString mSettingName;
  QComboBox * mpComboBox;

  std::vector<SItem> mItems;
};


using TSPCbElem = std::shared_ptr<CbElem>;

class CbElemColl
{
public:
  CbElemColl() = default;
  //~CbElemColl() = default;
  
  void set_setting_handler(QSettings *const ipS) { mpQSetting = ipS; }
  void read_cb_from_setting();
  
  void store_cb_elem(const TSPCbElem & ipCbElem);
  TSPCbElem operator[](const CbElem::ECbId iCbId);
  
private:
  using TMap = std::map<CbElem::ECbId, std::shared_ptr<CbElem> >;
  TMap mCbElems;   
  QSettings *mpQSetting = nullptr;
};


