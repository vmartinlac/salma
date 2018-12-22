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
    for(int i=0; i<vs->getNumberOfGenICamCameras(); i++)
    {
        addItem( vs->getNameOfGenICamCamera(i).c_str(), i );
    }
}

void CameraList::setSelectedCamera(const std::string& camid)
{
    if( camid.empty() == false )
    {
        VideoSystem* vs = VideoSystem::instance();    

        int sel = -1;
        for( int i = 0; sel < 0 && i<vs->getNumberOfGenICamCameras(); i++)
        {
            if( vs->getNameOfGenICamCamera(i) == camid )
            {
                sel = i;
            }
        }

        bool go_on = true;
        for(int i=0; go_on && i<count(); i++)
        {
            if(itemData(i).toInt() == sel)
            {
                setCurrentIndex(i);
                go_on = false;
            }
        }
    }
}

