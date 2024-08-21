#include "mypushbutton.h"
#include <QCoreApplication>
// #include <QEvent>
#include <QMouseEvent>
// #include <QTouchEvent>

MyPushButton::MyPushButton(QWidget *parent) : QPushButton(parent) {
    setAttribute(Qt::WA_AcceptTouchEvents, true);
}

bool MyPushButton::event(QEvent *e) {
    switch (e->type()) {
    case QEvent::TouchBegin: {
        QMouseEvent mouseEvent(QEvent::MouseButtonPress, QPointF(0, 0),
                               Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QCoreApplication::sendEvent(this, &mouseEvent);
        m_bTouchBegin = true;
        e->accept();
        return true;
    } break;
    case QEvent::TouchEnd: {
        QMouseEvent mouseEvent(QEvent::MouseButtonRelease, QPointF(0, 0),
                               Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QCoreApplication::sendEvent(this, &mouseEvent);
        m_bTouchEnd = true;
        e->accept();
        return true;
    } break;
    case QEvent::MouseButtonPress: {
        if (m_bTouchBegin) {
            m_bTouchBegin = false;
            e->accept();
            return true;
        }
    } break;
    case QEvent::MouseButtonRelease: {
        if (m_bTouchEnd) {
            m_bTouchEnd = false;
            e->accept();
            return true;
        }
    } break;
    default:
        break;
    }

    return QPushButton::event(e);
}
