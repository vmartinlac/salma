#include "CameraList.h"
#include "VideoSystem.h"

CameraList::CameraList(QWidget* parent) : QComboBox(parent)
{
    refresh();
}

int CameraList::getCameraId()
{
    bool ok;
    int ret = currentData().toInt(&ok);

    if(ok)
    {
        return ret;
    }
    else
    {
        return -1;
    }
}

void CameraList::refresh()
{
    clear();

    VideoSystem* vs = VideoSystem::instance();    
    for(int i=0; i<vs->getNumberOfAvtCameras(); i++)
    {
        addItem( vs->getNameOfAvtCamera(i).c_str(), i );
    }
}

