#include <QFile>
#include <QDebug>
#include <QJsonObject>
#include <QSettings>
#include "SLAMParameters.h"

bool SLAMParameters::loadFromJson(const QJsonDocument& doc)
{
    bool ret = false;

    if( doc.isObject() )
    {
        QJsonObject obj = doc.object();

        reset();

        cx = obj["cx"].toDouble(cx);
        cy = obj["cy"].toDouble(cy);
        fx = obj["fx"].toDouble(fx);
        fy = obj["fy"].toDouble(fy);
        distortion_k1 = obj["distortion_k1"].toDouble(distortion_k1);
        distortion_k2 = obj["distortion_k2"].toDouble(distortion_k2);
        distortion_k3 = obj["distortion_k3"].toDouble(distortion_k3);
        distortion_p1 = obj["distortion_p1"].toDouble(distortion_p1);
        distortion_p2 = obj["distortion_p2"].toDouble(distortion_p2);
        initialization_target_scale = obj["initialization_target_scale"].toDouble(initialization_target_scale);
        initialization_target_kind = obj["initialization_target_kind"].toInt(initialization_target_kind);
        patch_size = obj["patch_size"].toInt(patch_size);
        min_distance_to_camera = obj["min_distance_to_camera"].toDouble(min_distance_to_camera);
        max_landmark_candidates = obj["max_landmark_candidates"].toInt(max_landmark_candidates);
        num_depth_hypotheses = obj["num_depth_hypotheses"].toInt(num_depth_hypotheses);
        min_depth_hypothesis = obj["min_depth_hypothesis"].toDouble(min_depth_hypothesis);
        max_depth_hypothesis = obj["max_depth_hypothesis"].toDouble(max_depth_hypothesis);

        ret = true;
    }

    return ret;
}

bool SLAMParameters::saveToJson(QJsonDocument& doc)
{
    QJsonObject obj;

    obj["cx"] = cx;
    obj["cy"] = cy;
    obj["fx"] = fx;
    obj["fy"] = fy;
    obj["distortion_k1"] = distortion_k1;
    obj["distortion_k2"] = distortion_k2;
    obj["distortion_k3"] = distortion_k3;
    obj["distortion_p1"] = distortion_p1;
    obj["distortion_p2"] = distortion_p2;
    obj["initialization_target_scale"] = initialization_target_scale;
    obj["initialization_target_kind"] = initialization_target_kind;
    obj["patch_size"] = patch_size;
    obj["min_distance_to_camera"] = min_distance_to_camera;
    obj["max_landmark_candidates"] = max_landmark_candidates;
    obj["num_depth_hypotheses"] = num_depth_hypotheses;
    obj["min_depth_hypothesis"] = min_depth_hypothesis;
    obj["max_depth_hypothesis"] = max_depth_hypothesis;

    doc.setObject(obj);

    return true;
}

bool SLAMParameters::loadFromFile(const QString& path)
{
    QFile file(path);

    bool ok = file.open(QFile::ReadOnly);

    if(ok)
    {
        QByteArray content = file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(content);
        ok = loadFromJson(doc);
    }

    return ok;
}

bool SLAMParameters::saveToFile(const QString& path)
{
    QJsonDocument doc;
    bool ok = saveToJson(doc);

    if(ok)
    {
        QByteArray buffer = doc.toJson();

        QFile file(path);
        ok = file.open(QFile::WriteOnly);

        if(ok)
        {
            ok = ( file.write(buffer) == buffer.size() );
            file.close();
        }
    }

    return ok;
}

bool SLAMParameters::loadFromSettings()
{
    QSettings s;

    //qDebug() << s.fileName();

    s.beginGroup("slam");

    reset();

    cx = s.value("cx", cx).toDouble();
    cy = s.value("cy", cy).toDouble();
    fx = s.value("fx", fx).toDouble();
    fy = s.value("fy", fy).toDouble();
    distortion_k1 = s.value("distortion_k1", distortion_k1).toDouble();
    distortion_k2 = s.value("distortion_k2", distortion_k2).toDouble();
    distortion_k3 = s.value("distortion_k3", distortion_k3).toDouble();
    distortion_p1 = s.value("distortion_p1", distortion_p1).toDouble();
    distortion_p2 = s.value("distortion_p2", distortion_p2).toDouble();
    initialization_target_scale = s.value("initialization_target_scale", initialization_target_scale).toDouble();
    initialization_target_kind = s.value("initialization_target_kind", initialization_target_kind).toInt();
    patch_size = s.value("patch_size", patch_size).toInt();
    min_distance_to_camera = s.value("min_distance_to_camera", min_distance_to_camera).toDouble();
    max_landmark_candidates = s.value("max_landmark_candidates", max_landmark_candidates).toInt();
    num_depth_hypotheses = s.value("num_depth_hypotheses", num_depth_hypotheses).toInt();
    min_depth_hypothesis = s.value("min_depth_hypothesis", min_depth_hypothesis).toDouble();
    max_depth_hypothesis = s.value("max_depth_hypothesis", max_depth_hypothesis).toDouble();

    s.endGroup();

    return true;
}

bool SLAMParameters::saveToSettings()
{
    QSettings s;

    s.beginGroup("slam");

    s.setValue("cx", cx);
    s.setValue("cy", cy);
    s.setValue("fx", fx);
    s.setValue("fy", fy);
    s.setValue("distortion_k1", distortion_k1);
    s.setValue("distortion_k2", distortion_k2);
    s.setValue("distortion_k3", distortion_k3);
    s.setValue("distortion_p1", distortion_p1);
    s.setValue("distortion_p2", distortion_p2);
    s.setValue("initialization_target_scale", initialization_target_scale);
    s.setValue("initialization_target_kind", initialization_target_kind);
    s.setValue("patch_size", patch_size);
    s.setValue("min_distance_to_camera", min_distance_to_camera);
    s.setValue("max_landmark_candidates", max_landmark_candidates);
    s.setValue("num_depth_hypotheses", num_depth_hypotheses);
    s.setValue("min_depth_hypothesis", min_depth_hypothesis);
    s.setValue("max_depth_hypothesis", max_depth_hypothesis);

    s.endGroup();

    return true;
}

void SLAMParameters::reset()
{
    cx = 0.0;
    cy = 0.0;
    fx = 1.0;
    fy = 1.0;
    distortion_k1 = 0.0;
    distortion_k2 = 0.0;
    distortion_k3 = 0.0;
    distortion_p1 = 0.0;
    distortion_p2 = 0.0;
    initialization_target_scale = 1.0;
    initialization_target_kind = INITIALIZATION_TARGET_ONE_PLANE;
    patch_size = 11;
    min_distance_to_camera = 0.1;
    max_landmark_candidates = 30;
    num_depth_hypotheses = 100;
    min_depth_hypothesis = 0.2;
    max_depth_hypothesis = 5.0;
}

SLAMParameters::SLAMParameters()
{
    reset();
}

