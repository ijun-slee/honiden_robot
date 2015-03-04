#include <QtGui>
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#endif

#include "param_changer.h"
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

ParamChanger::ParamChanger(QWidget *parent, std::vector<std::string> param_name, std::vector<double> param_value) : QDialog(parent){
//parameterの名前と値のペアを代入する

  //パラメーターの数だけレイアウトを作成する。
  std::vector<QHBoxLayout*> layout_set;

  //パラメーターを設定する部分のレイアウト
    QVBoxLayout *parameterSetLayout = new QVBoxLayout;

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


  //ボタンのレイアウトを作成する
  QVBoxLayout *buttonLayout = new QVBoxLayout;
  QPushButton *changeButton = new QPushButton(tr("&Change"));
  QPushButton *closeButton = new QPushButton(tr("&Close"));
  buttonLayout->addWidget(changeButton); 
  buttonLayout->addWidget(closeButton);

    QHBoxLayout *paramBoxLayout = new QHBoxLayout;

  //layout_setのすべてのレイアウトを結合してひとつのレイアウトにする。
    QHBoxLayout *mainLayout = new QHBoxLayout;
    
    //changeButtonが押された時のコネクトの作成
    connect(changeButton, SIGNAL(clicked()),
	    this, SLOT(changeClicked()));
    
    for(int i = 0;i<param_name.size();i++){
//TODO記録したレイアウト全てを上から順に並べるようにする。　DONE
//TODO 現在横から並んでいるため、上から並べるようにレイアウトを変更する
//    mainLayout->addLayout(layout_set[i]);

    }
    mainLayout->addLayout(parameterSetLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);
}

void ParamChanger::setTopicTextCallback(const QString &text){
  //数字が変更された時のコールバック。
  //変更のボタンが可能になるように変更する関数を呼び出す。


}

void ParamChanger::changeClicked(){
  //changeButtonが押された時の挙動

}













