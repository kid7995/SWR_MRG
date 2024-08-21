#ifndef MYPUSHBUTTON_H
#define MYPUSHBUTTON_H

#include <QPushButton>

class MyPushButton : public QPushButton {
    Q_OBJECT
  public:
    explicit MyPushButton(QWidget *parent = nullptr);

  protected:
    bool event(QEvent *e);

  private:
    bool m_bTouchBegin = false;
    bool m_bTouchEnd = false;
};

#endif // MYPUSHBUTTON_H
