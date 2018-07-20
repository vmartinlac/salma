#include <QThread>

class Camera;
class SLAMEngine;

class SLAMThread : public QThread
{
public:

    SLAMThread(Camera* camera, SLAMEngine* engine);

protected:

    Camera* m_camera;
    SLAMEngine* m_engine;
};
