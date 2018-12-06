/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_BUTTON_GROUP_H
#define CNOID_BASE_BUTTON_GROUP_H

#include <cnoid/Signal>
#include <QButtonGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ButtonGroup : public QButtonGroup
{
    Q_OBJECT

public:
    ButtonGroup(QObject* parent = 0);

    SignalProxy<void(int id)> sigButtonClicked() {
        return sigButtonClicked_;
    }

private Q_SLOTS:
    void onButtonClicked(int id);

private:
    Signal<void(int id)> sigButtonClicked_;
};

}

#endif
