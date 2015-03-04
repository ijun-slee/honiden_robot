#ifndef FINDDIALOG_H
#define FINDDIALOG_H

#include <QDialog>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

class ParamChanger : public QDialog
{
    Q_OBJECT

public:
  ParamChanger(QWidget *parent , std::vector<std::string> param_name, std::vector<double> param_value);//parameterの名前と値のペアを代入する

signals:
  //    void findNext(const QString &str, Qt::CaseSensitivity cs);
  //   void findPrevious(const QString &str, Qt::CaseSensitivity cs);

private slots:
    void changeClicked();//変更が押された時の挙動
    //    void enablePublishButton();
    void setTopicTextCallback(const QString &text);
    //    void setTopicTypeCallback(const QString &text);


private:
    QLabel *label;
    QLabel *labelTopicType;
    QLineEdit *lineEdit;
    QLineEdit *lineEditType;
    QCheckBox *repeatCheckBox;
    QCheckBox *backwardCheckBox;
    QPushButton *changeButton;//変更が押された時のボタン
    QPushButton *closeButton;
    bool labelTopicTypeFlag;
    bool labelTopicTextFlag;



};

#endif










