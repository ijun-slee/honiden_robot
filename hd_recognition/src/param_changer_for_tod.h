#ifndef FINDDIALOG_H
#define FINDDIALOG_H

#include <QDialog>
#include "table_object_detector.h"


class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

class ParamChanger : public QDialog
{
    Q_OBJECT
      
public:
  ParamChanger(QWidget *parent, detector::TableObjectDetector &tableOD);
signals:
  //    void findNext(const QString &str, Qt::CaseSensitivity cs);
  //   void findPrevious(const QString &str, Qt::CaseSensitivity cs);


private slots:
    void changeClicked();//変更が押された時の挙動
    //    void enablePublishButton();
    void setTopicTextCallback(const QString &text);
    //    void setTopicTypeCallback(const QString &text);
    void setParamValueCallback(const QString &text, double &value);
    void valueLineEditLink(QLineEdit *lineEdit, double &value);
    QDoubleSpinBox* doubleSpinBoxCreate(double &default_val, double step_val, QString suffix,double decimal =1);
    void valueUpdate(/*double &updated_value,*/ double received_value);
    void setUpdateValue(double &update_value);
    void test(double x, double y);
    void connectSpinBoxToUpdateValue(double &update_value, QDoubleSpinBox* doubleBox);
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
    double update_value_token;
    std::vector<QDoubleSpinBox> updateDoubleSpinBoxes_;
    std::vector<double*> update_values_;
    std::vector<QDoubleSpinBox*> update_doublebox_;
    detector::TableObjectDetector* tableOD_ptr; 

};

#endif
















