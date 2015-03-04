#include <QtGui>
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#endif

#include "param_changer_for_tod.h"
#include <iostream>
#include <sstream>

//TODO
/*
 Kinectのview変更レイアウトの作成
 kinectのポジションパラメーター変更レイアウトの作成
 デバッグコンソールの作成→imageconverterを見ながらやると良い
 ボタンのイベントの作成



 Kinectのview変更レイアウトの作成
　→QGroupBoxなるものを使ってみるといいみたいだが…
 kinectのポジションパラメーター変更レイアウトの作成
 デバッグコンソールの作成→imageconverterを見ながらやると良い
 ボタンのイベントの作成



 */

ParamChanger::ParamChanger(QWidget *parent, detector::TableObjectDetector &tableOD) : QDialog(parent){
//parameterの名前と値のペアを代入する
  tableOD_ptr = &tableOD;
//キネクトの画角のためのGroupBoxを作成する
  QGroupBox* kinectViewGroupBox = new QGroupBox("Kinect View Parameter", this);

  //x, y, zのそれぞれのレイアウト
  QHBoxLayout *xparamLayout = new QHBoxLayout;
  QHBoxLayout *yparamLayout = new QHBoxLayout;
  QHBoxLayout *zparamLayout = new QHBoxLayout;

  QLabel *labelMax_x = new QLabel(tr("Max x"));//パラメーターの名前を示す
  QLabel *labelMin_x = new QLabel(tr("Min x"));//パラメーターの名前を示す
  QLabel *labelMax_y = new QLabel(tr("Max y"));//パラメーターの名前を示す
  QLabel *labelMin_y = new QLabel(tr("Min y"));//パラメーターの名前を示す
  QLabel *labelMax_z = new QLabel(tr("Max z"));//パラメーターの名前を示す
  QLabel *labelMin_z = new QLabel(tr("Min z"));//パラメーターの名前を示す

  QLineEdit *lineEditMax_x = new QLineEdit;//パラメーターの値を記入する    
  QLineEdit *lineEditMin_x = new QLineEdit;//パラメーターの値を記入する    
  QLineEdit *lineEditMax_y = new QLineEdit;//パラメーターの値を記入する    
  QLineEdit *lineEditMin_y = new QLineEdit;//パラメーターの値を記入する    
  QLineEdit *lineEditMax_z = new QLineEdit;//パラメーターの値を記入する    
  QLineEdit *lineEditMin_z = new QLineEdit;//パラメーターの値を記入する    
    //エディットの値が変化した時のコネクトを用意する。

  
  //  QDoubleSpinBox *doubleSpinMax_x = new QDoubleSpinBox;
  // std::stringstream val;
  // val<<0.01;
  //  doubleSpinMax_x->setValue(1.11);
  // valueLineEditLink(lineEditMax_x,tableOD.detectfield.max_x);
  // valueLineEditLink(lineEditMin_x,tableOD.detectfield.min_x);
  // valueLineEditLink(lineEditMax_y,tableOD.detectfield.max_y);
  // valueLineEditLink(lineEditMin_y,tableOD.detectfield.min_y);
  // valueLineEditLink(lineEditMax_z,tableOD.detectfield.max_z);
  // valueLineEditLink(lineEditMin_z,tableOD.detectfield.min_z);


 
 QString suffix_string = " cm";
  //レイアウトの作成
 //TODO Createの2番目の引数をtableOD.detectfield.max_xに変更する
 //TODO このSpinBoxで変更された値が、実際のdetectfieldに反映されるような紐付を行う。
/*
  xparamLayout->addWidget(labelMax_x);
  xparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.max_x, 1.0, suffix_string));
  xparamLayout->addWidget(labelMin_x);
  xparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.min_x, 1.0, suffix_string));

  yparamLayout->addWidget(labelMax_y);
  yparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.max_y, 1.0, suffix_string));
  yparamLayout->addWidget(labelMin_y);
  yparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.min_y, 1.0, suffix_string));

  zparamLayout->addWidget(labelMax_z);
  zparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.max_z, 1.0, suffix_string));
  zparamLayout->addWidget(labelMin_z);
  zparamLayout->addWidget(doubleSpinBoxCreate(tableOD.detectfield.min_z, 1.0, suffix_string));
x
*/
//各値へのDoubleSpinBoxの作成
// QDoubleSpinBox *doubleBoxMax_x = new QDoubleSpinBox(doubleSpinBoxCreate(tableOD.detectfield.max_x, 1.0, suffix_string) );
 QDoubleSpinBox *doubleBoxMax_x = doubleSpinBoxCreate(tableOD_ptr->detectfield.max_x, 1.0, suffix_string);
 QDoubleSpinBox *doubleBoxMax_y = doubleSpinBoxCreate(tableOD.detectfield.max_y, 1.0, suffix_string);
 QDoubleSpinBox *doubleBoxMax_z = doubleSpinBoxCreate(tableOD.detectfield.max_z, 1.0, suffix_string) ;
 QDoubleSpinBox *doubleBoxMin_x = doubleSpinBoxCreate(tableOD.detectfield.min_x, 1.0, suffix_string) ;
 QDoubleSpinBox *doubleBoxMin_y = doubleSpinBoxCreate(tableOD.detectfield.min_y, 1.0, suffix_string) ;
 QDoubleSpinBox *doubleBoxMin_z = doubleSpinBoxCreate(tableOD.detectfield.min_z, 1.0, suffix_string);

 xparamLayout->addWidget(labelMax_x);
 xparamLayout->addWidget(doubleBoxMax_x);
 xparamLayout->addWidget(labelMin_x);
 xparamLayout->addWidget(doubleBoxMin_x);
 
 yparamLayout->addWidget(labelMax_y);
 yparamLayout->addWidget(doubleBoxMax_y);
 yparamLayout->addWidget(labelMin_y);
 yparamLayout->addWidget(doubleBoxMin_y);
 
 zparamLayout->addWidget(labelMax_z);
 zparamLayout->addWidget(doubleBoxMax_z);
 zparamLayout->addWidget(labelMin_z);
 zparamLayout->addWidget(doubleBoxMin_z); 
 
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.max_x, doubleBoxMax_x);
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.min_x, doubleBoxMin_x);
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.max_y, doubleBoxMax_y);
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.min_y, doubleBoxMin_y);
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.max_z, doubleBoxMax_z);
 connectSpinBoxToUpdateValue(tableOD_ptr->detectfield.min_z, doubleBoxMin_z);

  //layout_setのすべてのレイアウトを結合してひとつのレイアウトにする。
    QVBoxLayout *mainLayout = new QVBoxLayout;

    //kinectの画角のパラメーターを設定するためのレイアウト
    QVBoxLayout *kinectViewVBox = new QVBoxLayout;
    kinectViewVBox->addLayout(xparamLayout);
    kinectViewVBox->addLayout(yparamLayout);
    kinectViewVBox->addLayout(zparamLayout);

    //ChangeとCloseを設定するためのレイアウト
    QHBoxLayout *buttonVBox = new QHBoxLayout;
    QPushButton *changeButton = new QPushButton(tr("&Change"));
    QPushButton *closeButton = new QPushButton(tr("&Close"));

    //ButtonVBoxに追加
    buttonVBox->addWidget(changeButton);
    buttonVBox->addWidget(closeButton);

    //コールバック関数の設定
    connect(changeButton, SIGNAL(clicked()),
	    this, SLOT(changeClicked()));
    connect(closeButton, SIGNAL(clicked()),
	    this, SLOT(close()));



    kinectViewGroupBox->setLayout(kinectViewVBox);    
    mainLayout->addWidget(kinectViewGroupBox);
    mainLayout->addLayout(buttonVBox);
    //    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);
    /*
  //パラメーターを設定する部分のレイアウト
  QVBoxLayout *parameterSetLayout = new QVBoxLayout;



    /*
  for(int i = 0;i<param_name.size();i++){
    QHBoxLayout *layout = new QHBoxLayout;
    std::string labelstring = "Parameter: " + param_name[i];
    QLabel *label = new QLabel(tr(labelstring.c_str()));//パラメーターの名前を示す
    QLineEdit *lineEdit = new QLineEdit;//パラメーターの値を記入する    
    //エディットの値が変化した時のコネクトを用意する。
    connect(lineEdit, SIGNAL(textChanged(const QString &)),
            this, SLOT(setTopicTextCallback(const QString &)));   
    
    //TODO　出来ればスライドを入れるようにしたいが。。。
    layout->addWidget(label); 
    layout->addWidget(lineEdit);
    //    layout.addWidget(&changeButton);

    
    //TODO ボタンをつける処理を行うようにする。 
    layout_set.push_back(layout);//作成したレイアウトをコンテナに入れる。
    parameterSetLayout->addLayout(layout);
  }

    */
    /*
  //ボタンのレイアウトを作成する
  QVBoxLayout *buttonLayout = new QVBoxLayout;
  QPushButton *changeButton = new QPushButton(tr("&Change"));
  QPushButton *closeButton = new QPushButton(tr("&Close"));
  buttonLayout->addWidget(changeButton); 
  buttonLayout->addWidget(closeButton);

    QHBoxLayout *paramBoxLayout = new QHBoxLayout;

    
    //changeButtonが押された時のコネクトの作成
    connect(changeButton, SIGNAL(clicked()),
	    this, SLOT(changeClicked()));
    
  //TODO記録したレイアウト全てを上から順に並べるようにする。　DONE
//TODO 現在横から並んでいるため、上から並べるようにレイアウトを変更する
//    mainLayout->addLayout(layout_set[i]);

    mainLayout->addLayout(parameterSetLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);
*/

}

void ParamChanger::setTopicTextCallback(const QString &text){
  //数字が変更された時のコールバック。
  //変更のボタンが可能になるように変更する関数を呼び出す。
  

}

void ParamChanger::setParamValueCallback(const QString &text, double &value){
  //数字が変更された時のコールバック。
  //変更のボタンが可能になるように変更する関数を呼び出す。


}

void ParamChanger::changeClicked(){
  //changeButtonが押された時の挙動
  std::cout<<"Change Clicked"<<std::endl;
   std::cout<<tableOD_ptr->detectfield.max_x<<std::endl;
  //クリックされた時にLineEditに入っている値をとってきて、その値で更新する処理にする。
}

void ParamChanger::valueLineEditLink(QLineEdit *lineEdit, double &value){
  //LineEditと値をひもづけして、LineEditを変更することで値が変更するような紐付を行う
 
  //この方法じゃあだめだな。
  //   connect(lineEdit, SIGNAL(textChanged(const QString &)),
  //	   this, SLOT(setParamValueCallback(const QString &,double &)));   
    //SLOT(setTopicTextCallback(const QString &)));   
  //TODOTest エディットをすると値が変化しているかどうかを確認する。


}

//doubleSpinBoxを作成する関数。引数は、初期値、
QDoubleSpinBox* ParamChanger::doubleSpinBoxCreate(double &default_val, double step_val, QString suffix, double decimal){
  QDoubleSpinBox *doubleSpinB = new QDoubleSpinBox;
  doubleSpinB->setValue(default_val);
  doubleSpinB->setSingleStep(step_val);
  doubleSpinB->setDecimals(decimal);
  doubleSpinB->setSuffix(suffix);

  //値が変化するごとにその値も更新するようなコネクト
  /*これだとコンパイル通る
  connect(doubleSpinB, SIGNAL(valueChanged(double)),
	  this, SLOT(valueUpdate(  double )));
  */
  //  boost::function<void(double)> func = boost::bind<void,double,double>(&test, 2.0, _1);
  connect(doubleSpinB, SIGNAL(valueChanged(double)),
	  	  this, SLOT(valueUpdate( double)));
	  //  this, SLOT(func( double)));
  return doubleSpinB;
}
			 
			 

void ParamChanger::valueUpdate(/*double &updated_value,*/ double received_value){
  //  std::cout<<"Value Update: "<<received_value<<std::endl;
  for(int i = 0;i<update_values_.size();i++){
    *update_values_[i] = update_doublebox_[i]->value();
    //  std::cout<<"Value Update: "<<i<<" "<<update_values_[i]<<std::endl;
  }


}
void ParamChanger::setUpdateValue(double &update_value){
  //  update_value_token = &update_value;
}


void ParamChanger::test(double x, double y){



}

void ParamChanger::connectSpinBoxToUpdateValue(double &update_value, QDoubleSpinBox* doubleBox){
  //どのdoubleBoxがどの値のアップデートに関連しているかを登録する。
  update_values_.push_back(&update_value);
  update_doublebox_.push_back(doubleBox);



}






