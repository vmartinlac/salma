#include "Project.h"
#include "Model.h"

Model::Model(Project* proj) : QAbstractListModel(proj)
{
    mProject = proj;
}

Project* Model::project()
{
    return mProject;
}

