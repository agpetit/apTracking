#include "modelloader.h"
#include <osgDB/ReadFile>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Group>
#include <osg/Array>
#include <osg/Transform>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Material>
#include "utils.h"
#include <fstream>
#include <cstring>
#include <cstdio>
#include <map>
#include <iostream>
#include <clocale>
#include <unordered_map>

using namespace osg;
using namespace std;

namespace std
{
    template<class A, class B, class C>
    class hash<luxifer::tuple<A,B,C> >
    {
    public:
        inline size_t operator()(const luxifer::tuple<A,B,C> &v) const
        {
            size_t h(0);
            const unsigned char *data = (const unsigned char*)&v;
            for(size_t i = 0 ; i < sizeof(v) ; ++i)
                h = ((h << 5) - h) ^ data[i];
            return h;
        }
    };
}

namespace luxifer
{
    template<class K, class V>
    class map : public std::unordered_map<K, V>
    {
    };

    ModelLoader::Material::Material()
        : ambient(0,0,0,0),
          diffuse(1,1,1,1),
          specular(0,0,0,0),
          emissive(0,0,0,0),
          sharpness(0),
          Ns(1)
    {

    }

    ModelLoader::ModelLoader()
    {
    }

    ref_ptr<Node> ModelLoader::load(const string &filename)
    {
        if (filename.empty())   return NULL;
        if (filename.size() > 3 && lower(right(filename, 4)) == ".obj")
        {
            ModelLoader loader;
            return loader.loadOBJ(filename);
        }
        if (filename.size() > 3 && lower(right(filename, 4)) == ".off")
        {
            ModelLoader loader;
            return loader.loadOFF(filename);
        }        return osgDB::readNodeFile(filename);
    }

    ref_ptr<Node> ModelLoader::load(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
    {

            ModelLoader loader;
            return loader.loadExt(vertices, normals, triangles);
    }


    ref_ptr<Node> ModelLoader::loadExt(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
    {


        ref_ptr<Group> model = new Group;
        model->setName("model");

        ref_ptr<Group> group = new Group;
        model->addChild(group);

        ref_ptr<Geode> geode = new Geode;
        group->addChild(geode);

        ref_ptr<Geometry> geom = new Geometry;
        geode->addDrawable(geom);

        ref_ptr<Vec3Array> v_vertex = new Vec3Array;
        ref_ptr<Vec3Array> v_normal = new Vec3Array;
        ref_ptr<Vec2Array> v_tcoord = new Vec2Array;
        ref_ptr<DrawElementsUInt> v_index = new DrawElementsUInt(GL_TRIANGLES);

        geom->setVertexArray(v_vertex);
        geom->setNormalArray(v_normal);
        geom->setTexCoordArray(0, v_tcoord);
        geom->addPrimitiveSet(v_index);

        string line;
        string mtlname;

        materials[mtlname] = Material();
        setMaterial(mtlname, geom);

        vector<Vec3> vertex;
        vector<Vec3> normal;
        vector<Vec2> tcoord;

        vector<unsigned int> idx;
        vector<bool> normal_is_null;
        map<tuple<int,int,int>, unsigned int> m_vtx;

        bool b_smooth = true;

        for (int i = 0 ; i < vertices.size(); i++)
        {
            vertex.push_back(Vec3(vertices[i].x, vertices[i].y, vertices[i].z));
            normal.push_back(Vec3(normals[i].x, normals[i].y, normals[i].z));
        }

        std::cout << " size vertices " << vertices.size() << " size triangles " << triangles.size() << std::endl;

        for (int i = 0 ; i < triangles.size(); i++)
        {
          idx.clear();
          int v1, vt1, vn1;
          int v2, vt2, vn2;
          int v3, vt3, vn3;

          vt1 = -1;
          v1 = triangles[i].v1;
          vn1 = triangles[i].n1;

          vt2 = -1;
          v2 = triangles[i].v2;
          vn2 = triangles[i].n2;

          vt3 = -1;
          v3 = triangles[i].v3;
          vn3 = triangles[i].n3;

         //std::cout << " v1 " << v1 << " vn1 " << vn1 << std::endl;

          const tuple<int,int,int> tp1 = make_tuple(v1, vt1, vn1);
          map<tuple<int,int,int>, unsigned int >::const_iterator it1 = b_smooth ? m_vtx.find(tp1) : m_vtx.end();
          if (it1 == m_vtx.end())
          {
              if (b_smooth)
                  m_vtx[tp1] = v_vertex->size();
              idx.push_back(v_vertex->size());
              v_vertex->push_back(vertex[v1]);
              if (vt1 >= 0)
                  v_tcoord->push_back(tcoord[vt1]);
              else
                  v_tcoord->push_back(Vec2(0,0));
              if (vn1 >= 0)
              {
                  v_normal->push_back(normal[vn1]);
                  normal_is_null.push_back(false);
              }
              else
              {
                  normal_is_null.push_back(true);
                  v_normal->push_back(Vec3(0,0,0));
              }

              //std::cout << " v1 " << v1 << " vn1 " << vn1 << std::endl;
          }
          else
              idx.push_back(it1->second);

          const tuple<int,int,int> tp2 = make_tuple(v2, vt2, vn2);
          map<tuple<int,int,int>, unsigned int >::const_iterator it2 = b_smooth ? m_vtx.find(tp2) : m_vtx.end();
          if (it2 == m_vtx.end())
          {
              if (b_smooth)
                  m_vtx[tp2] = v_vertex->size();
              idx.push_back(v_vertex->size());
              v_vertex->push_back(vertex[v2]);
              if (vt2 >= 0)
                  v_tcoord->push_back(tcoord[vt2]);
              else
                  v_tcoord->push_back(Vec2(0,0));
              if (vn2 >= 0)
              {
                  v_normal->push_back(normal[vn2]);
                  normal_is_null.push_back(false);
              }
              else
              {
                  normal_is_null.push_back(true);
                  v_normal->push_back(Vec3(0,0,0));
              }
          }
          else
              idx.push_back(it2->second);

          const tuple<int,int,int> tp3 = make_tuple(v3, vt3, vn3);
          map<tuple<int,int,int>, unsigned int >::const_iterator it3 = b_smooth ? m_vtx.find(tp3) : m_vtx.end();
          if (it3 == m_vtx.end())
          {
              if (b_smooth)
                  m_vtx[tp3] = v_vertex->size();
              idx.push_back(v_vertex->size());
              v_vertex->push_back(vertex[v3]);
              if (vt3 >= 0)
                  v_tcoord->push_back(tcoord[vt3]);
              else
                  v_tcoord->push_back(Vec2(0,0));
              if (vn3 >= 0)
              {
                  v_normal->push_back(normal[vn3]);
                  normal_is_null.push_back(false);
              }
              else
              {
                  normal_is_null.push_back(true);
                  v_normal->push_back(Vec3(0,0,0));
              }
          }
          else
              idx.push_back(it3->second);

          for(size_t i = 2 ; i < idx.size() ; ++i)
          {
              v_index->addElement(idx[0]);
              v_index->addElement(idx[i - 1]);
              v_index->addElement(idx[i]);
          }



        }

        std::cout << " ok load mesh " << v_vertex->size() << " " << v_normal->size() << std::endl;

        const string object_name = "deformablemesh";

                    /*if (geode->getNumDrawables() == 1 && geom->getNumPrimitiveSets() == 1 && v_index->getNumIndices() == 0)
                    {
                        geode->setName(object_name);
                    }
                    else
                    {
                        postProcess(v_vertex, v_normal, v_tcoord, v_index, normal_is_null);

                        geode = new Geode;
                        geode->setName(object_name);
                        group->addChild(geode);

                        geom = new Geometry;
                        geode->addDrawable(geom);

                        v_vertex = new Vec3Array;
                        v_normal = new Vec3Array;
                        v_tcoord = new Vec2Array;
                        v_index = new DrawElementsUInt(GL_TRIANGLES);

                        geom->setVertexArray(v_vertex);
                        geom->setNormalArray(v_normal);
                        geom->setTexCoordArray(0, v_tcoord);
                        geom->addPrimitiveSet(v_index);
                    }
                    normal_is_null.clear();
                    m_vtx.clear();
                    setMaterial(mtlname, geom);*/

        postProcess(v_vertex, v_normal, v_tcoord, v_index, normal_is_null);

        //model->computeBound();

        return model;
    }

    ref_ptr<Node> ModelLoader::loadOFF(const std::string &filename)
    {
        ref_ptr<Group> model = new Group;
        model->setName(filename);

        ref_ptr<Group> group = new Group;
        model->addChild(group);

        ref_ptr<Geode> geode = new Geode;
        group->addChild(geode);

        ref_ptr<Geometry> geom = new Geometry;
        geode->addDrawable(geom);

        ref_ptr<Vec3Array> v_vertex = new Vec3Array;
        ref_ptr<Vec3Array> v_normal = new Vec3Array;
        ref_ptr<Vec2Array> v_tcoord = new Vec2Array;
        ref_ptr<DrawElementsUInt> v_index = new DrawElementsUInt(GL_TRIANGLES);

        geom->setVertexArray(v_vertex);
        geom->setNormalArray(v_normal);
        geom->setTexCoordArray(0, v_tcoord);
        geom->addPrimitiveSet(v_index);

        ifstream in(filename.c_str(), ios_base::in);

        string type;
        in >> type;

        int nb_vertices, nb_indexes;
        in >> nb_vertices >> nb_indexes;
        const bool coff = (type == "COFF");
        int tmp;
        in >> tmp;

        v_vertex->reserve(nb_vertices);
        v_index->reserve(nb_indexes);
        for(size_t i = 0 ; i < nb_vertices ; ++i)
        {
            Vec3d v;
            double c;
            in >> v.x() >> v.y() >> v.z();
            if (coff)
                in >> c >> c >> c >> c;
            v_vertex->push_back(v);
        }
        v_tcoord->resize(v_vertex->size(), Vec2d(0,0));
        v_normal->resize(v_vertex->size(), Vec3d(0,0,0));

        for(size_t i = 0 ; i < nb_indexes ; ++i)
        {
            int n, a, b, c;
            in >> n >> a >> b >> c;
            while(in.get() != '\n' && !in.eof())  {}
            v_index->push_back(a);
            v_index->push_back(b);
            v_index->push_back(c);
        }

        postProcess(v_vertex, v_normal, v_tcoord, v_index, std::vector<bool>());

        model->computeBound();

        return model;
    }

    ref_ptr<Node> ModelLoader::loadOBJ(const std::string &filename)
    {
        ifstream file(filename.c_str(), ios_base::in);
        if (!file)
            return NULL;

        ref_ptr<Group> model = new Group;
        model->setName(filename);

        ref_ptr<Group> group = new Group;
        model->addChild(group);

        ref_ptr<Geode> geode = new Geode;
        group->addChild(geode);

        ref_ptr<Geometry> geom = new Geometry;
        geode->addDrawable(geom);

        ref_ptr<Vec3Array> v_vertex = new Vec3Array;
        ref_ptr<Vec3Array> v_normal = new Vec3Array;
        ref_ptr<Vec2Array> v_tcoord = new Vec2Array;
        ref_ptr<DrawElementsUInt> v_index = new DrawElementsUInt(GL_TRIANGLES);

        geom->setVertexArray(v_vertex);
        geom->setNormalArray(v_normal);
        geom->setTexCoordArray(0, v_tcoord);
        geom->addPrimitiveSet(v_index);

        string line;
        string mtlname;

        materials[mtlname] = Material();
        setMaterial(mtlname, geom);

        vector<Vec3> vertex;
        vector<Vec3> normal;
        vector<Vec2> tcoord;

        vector<unsigned int> idx;
        vector<bool> normal_is_null;
        map<tuple<int,int,int>, unsigned int> m_vtx;

        bool b_smooth = true;

        while(getline(file, line) && !file.eof())
        {
            // Skip empty lines
            if (line.empty())
                continue;

            switch(line[0])
            {
            case '#':
                break;
            case 'v':       // Vertex data
                {
                    float x, y, z, w;
                    int n;
                    // Points
                    if ((n = parseFormat(line.c_str(), "v %f %f %f %f", x, y, z, w)) > 0)
                    {
                        switch(n)
                        {
                        case 1: vertex.push_back(Vec3(x,0.f,0.f));  break;
                        case 2: vertex.push_back(Vec3(x,y,0.f));  break;
                        case 3: vertex.push_back(Vec3(x,y,z));  break;
                        case 4: vertex.push_back(Vec3(x/w,y/w,z/w));  break;
                        }
                    }
                    // Normals
                    else if ((n = parseFormat(line.c_str(), "vn %f %f %f", x, y, z)) > 0)
                    {
                        switch(n)
                        {
                        case 1: normal.push_back(Vec3(x,0.f,0.f));  break;
                        case 2: normal.push_back(Vec3(x,y,0.f));  break;
                        case 3: normal.push_back(Vec3(x,y,z));  break;
                        }
                    }
                    // Texture coordinates
                    else if ((n = parseFormat(line.c_str(), "vt %f %f %f", x, y, z)) > 0)
                    {
                        switch(n)
                        {
                        case 1: tcoord.push_back(Vec2(x,0.f));  break;
                        case 2:
                        case 3: tcoord.push_back(Vec2(x,y));  break;
                        }
                    }
                }
                break;
            case 'f':
                if (left(line, 2) == "f ")
                {
                    size_t pos = 2;
                    idx.clear();
                    const size_t line_length = line.size();
                    const char *p_data = line.c_str();
                    while(pos < line_length)
                    {
                        while(pos < line_length && p_data[pos] == ' ')
                            ++pos;
                        if (pos == line_length)
                            break;
                        int v, vt, vn;
                        int n;
                        if (parseFormat(p_data + pos, "%d//%d", v, vn) == 2)
                        {
                            if (v < 0)  v += vertex.size();
                            if (vn < 0) vn += normal.size();
                            vt = -1;
                        }
                        else if ((n = parseFormat(p_data + pos, "%d/%d/%d", v, vt, vn)) > 0)
                        {
                            switch(n)
                            {
                            case 1:
                                if (v < 0)  v += vertex.size();
                                vt = -1;
                                vn = -1;
                                break;
                            case 2:
                                if (v < 0)  v += vertex.size();
                                if (vt < 0) vt += tcoord.size();
                                vn = -1;
                                break;
                            case 3:
                                if (v < 0)  v += vertex.size();
                                if (vn < 0) vn += normal.size();
                                if (vt < 0) vt += tcoord.size();
                                break;
                            }
                        }
                        else
                            break;
                        const tuple<int,int,int> tp = make_tuple(v, vt, vn);
                        map<tuple<int,int,int>, unsigned int >::const_iterator it = b_smooth ? m_vtx.find(tp) : m_vtx.end();
                        if (it == m_vtx.end())
                        {
                            if (b_smooth)
                                m_vtx[tp] = v_vertex->size();
                            idx.push_back(v_vertex->size());
                            --v;
                            --vt;
                            --vn;
                            v_vertex->push_back(vertex[v]);
                            if (vt >= 0)
                                v_tcoord->push_back(tcoord[vt]);
                            else
                                v_tcoord->push_back(Vec2(0,0));
                            if (vn >= 0)
                            {
                                v_normal->push_back(normal[vn]);
                                normal_is_null.push_back(false);
                            }
                            else
                            {
                                normal_is_null.push_back(true);
                                v_normal->push_back(Vec3(0,0,0));
                            }
                        }
                        else
                            idx.push_back(it->second);

                        while(pos < line_length && p_data[pos] != ' ')
                            ++pos;
                    }
                    if (idx.size() < 3)
                        break;
                    // Triangulate
                    for(size_t i = 2 ; i < idx.size() ; ++i)
                    {
                        v_index->addElement(idx[0]);
                        v_index->addElement(idx[i - 1]);
                        v_index->addElement(idx[i]);
                    }
                }
                break;
            case 'l':
            case 'p':
                break;
            case 's':
                if (line == "s off")
                {
                    b_smooth = false;
                }
                else if (left(line, 2) == "s ")
                {
                    const string smoothing_group_name = line.substr(2);
                    b_smooth = true;
                }
                break;
            case 'g':
//                if (left(line, 2) == "g ")
//                {
//                    const string group_name = line.substr(2);
//                    group->setName(group_name);
//                }
                break;
            case 'o':
                if (left(line, 2) == "o ")
                {
                    const string object_name = line.substr(2);

                    if (geode->getNumDrawables() == 1 && geom->getNumPrimitiveSets() == 1 && v_index->getNumIndices() == 0)
                    {
                        geode->setName(object_name);
                    }
                    else
                    {
                        postProcess(v_vertex, v_normal, v_tcoord, v_index, normal_is_null);

                        geode = new Geode;
                        geode->setName(object_name);
                        group->addChild(geode);

                        geom = new Geometry;
                        geode->addDrawable(geom);

                        v_vertex = new Vec3Array;
                        v_normal = new Vec3Array;
                        v_tcoord = new Vec2Array;
                        v_index = new DrawElementsUInt(GL_TRIANGLES);

                        geom->setVertexArray(v_vertex);
                        geom->setNormalArray(v_normal);
                        geom->setTexCoordArray(0, v_tcoord);
                        geom->addPrimitiveSet(v_index);
                    }
                    normal_is_null.clear();
                    m_vtx.clear();
                    setMaterial(mtlname, geom);
                }
                break;
            case 'u':
                if (left(line, 7) == "usemtl ")
                {
                    mtlname = line.substr(7);

                    if (geom->getNumPrimitiveSets() == 1 && v_index->getNumIndices() == 0)
                    {
                    }
                    else
                    {
                        postProcess(v_vertex, v_normal, v_tcoord, v_index, normal_is_null);

                        geom = new Geometry;
                        geode->addDrawable(geom);

                        v_vertex = new Vec3Array;
                        v_normal = new Vec3Array;
                        v_tcoord = new Vec2Array;
                        v_index = new DrawElementsUInt(GL_TRIANGLES);

                        geom->setVertexArray(v_vertex);
                        geom->setNormalArray(v_normal);
                        geom->setTexCoordArray(0, v_tcoord);
                        geom->addPrimitiveSet(v_index);
                    }
                    normal_is_null.clear();
                    m_vtx.clear();
                    setMaterial(mtlname, geom);
                }
                break;
            case 'm':
                if (left(line, 7) == "mtllib ")
                {
                    const string libname = line.substr(7);
                    loadMaterialLib(makePathRelativeTo(libname, extractPath(filename)));
                }
                break;
            }
            line.clear();
        }

        postProcess(v_vertex, v_normal, v_tcoord, v_index, normal_is_null);

        model->computeBound();

        return model;
    }

    void ModelLoader::postProcess(ref_ptr<Vec3Array> v_vertex,
                                  ref_ptr<Vec3Array> v_normal,
                                  ref_ptr<Vec2Array> v_tcoord,
                                  ref_ptr<DrawElementsUInt> v_index,
                                  const vector<bool> &normal_is_null)
    {
        for(size_t i = 0 ; i < v_normal->size() ; ++i)
            (*v_normal)[i] = Vec3(0,0,0);
        for(size_t i = 0 ; i < v_index->getNumIndices() ; i += 3)
        {
            const unsigned int a = v_index->getElement(i);
            const unsigned int b = v_index->getElement(i + 1);
            const unsigned int c = v_index->getElement(i + 2);
            const Vec3d &ab = Vec3d((*v_vertex)[b]) - Vec3d((*v_vertex)[a]);
            const Vec3d &ac = Vec3d((*v_vertex)[c]) - Vec3d((*v_vertex)[a]);
            Vec3d n = ab ^ ac;
            n.normalize();
            (*v_normal)[a] += n;
            (*v_normal)[b] += n;
            (*v_normal)[c] += n;
        }

        for(size_t i = 0 ; i < v_normal->size() ; ++i)
            (*v_normal)[i].normalize();
    }

    void ModelLoader::setMaterial(const std::string &mtlname, osg::ref_ptr<osg::Geometry> geom)
    {
        const Material &mat = materials[mtlname];
        osg::StateSet *stateset = new osg::StateSet;
        geom->setStateSet(stateset);
        stateset->setName(mtlname);

        if (mat.diffuse_map)
            stateset->setTextureAttributeAndModes(0, mat.diffuse_map);
        if (mat.specular_map)
            stateset->setTextureAttributeAndModes(1, mat.specular_map);
        if (mat.normal_map)
            stateset->setTextureAttributeAndModes(2, mat.normal_map);
        if (mat.emission_map)
            stateset->setTextureAttributeAndModes(3, mat.emission_map);

        osg::ref_ptr<osg::Material> osg_mat = new osg::Material;
        osg_mat->setAmbient(osg::Material::FRONT_AND_BACK, mat.ambient);
        osg_mat->setDiffuse(osg::Material::FRONT_AND_BACK, mat.diffuse);
        osg_mat->setSpecular(osg::Material::FRONT_AND_BACK, mat.specular);
        osg_mat->setEmission(osg::Material::FRONT_AND_BACK, mat.emissive);
        osg_mat->setShininess(osg::Material::FRONT_AND_BACK, min(128.0, mat.Ns));
        stateset->setAttribute(osg_mat, StateAttribute::ON);
    }

    void ModelLoader::loadMaterialLib(const std::string &libname)
    {
        ifstream file(libname.c_str(), ios_base::in);
        if (!file)
            return;

        string line;

        string current("default");
        float r, g, b, a;
        int n;

        while(getline(file, line) && !file.eof())
        {
            if (line.empty())
                continue;

            switch(line[0])
            {
            case '#':
                break;
            case 'n':
                if (left(line, 7) == "newmtl ")
                {
                    current = line.substr(7);
                }
                break;
            case 'N':
                if (parseFormat(line, "Ns %f", r))
                {
                    materials[current].Ns = r;
                }
                else if (parseFormat(line, "Ni %f", r))
                {
                }
                break;
            case 'K':
                if ((n = parseFormat(line, "Ka %f %f %f %f", r, g, b, a)) > 0)
                {
                    switch(n)
                    {
                    case 1: g = 0.0;
                    case 2: b = 0.0;
                    case 3: a = 0.0;
                    case 4:
                        break;
                    }
                    materials[current].ambient = Vec4d(r, g, b, a);
                }
                else if ((n = parseFormat(line, "Kd %f %f %f %f", r, g, b, a)) > 0)
                {
                    switch(n)
                    {
                    case 1: g = 0.0;
                    case 2: b = 0.0;
                    case 3: a = 0.0;
                    case 4:
                        break;
                    }
                    materials[current].diffuse = Vec4d(r, g, b, a);
                }
                else if ((n = parseFormat(line, "Ks %f %f %f %f", r, g, b, a)) > 0)
                {
                    switch(n)
                    {
                    case 1: g = 0.0;
                    case 2: b = 0.0;
                    case 3: a = 0.0;
                    case 4:
                        break;
                    }
                    materials[current].specular = Vec4d(r, g, b, a);
                }
                else if ((n = parseFormat(line, "Ke %f %f %f %f", r, g, b, a)) > 0)
                {
                    switch(n)
                    {
                    case 1: g = 0.0;
                    case 2: b = 0.0;
                    case 3: a = 0.0;
                    case 4:
                        break;
                    }
                    materials[current].emissive = Vec4d(r, g, b, a);
                }
                break;
            case 'd':
                break;
            case 'i':
                break;
            case 'm':
                if (left(line, 7) == "map_Kd ")
                {
                    const string filename = makePathRelativeTo(line.substr(7), extractPath(libname));
                    Image *image = osgDB::readImageFile(filename);
                    if (image)
                    {
                        Texture2D *tex = new Texture2D(image);
                        tex->setWrap(Texture2D::WRAP_R, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
                        materials[current].diffuse_map = tex;
                    }
                }
                else if (left(line, 7) == "map_Ka ")
                {
                    const string filename = makePathRelativeTo(line.substr(7), extractPath(libname));
//                    Image *image = osgDB::readImageFile(filename);
//                    if (image)
//                    materials[current].ambient_map = new osg::Texture2D(image);
                }
                else if (left(line, 7) == "map_Ks ")
                {
                    const string filename = makePathRelativeTo(line.substr(7), extractPath(libname));
                    Image *image = osgDB::readImageFile(filename);
                    if (image)
                    {
                        Texture2D *tex = new Texture2D(image);
                        tex->setWrap(Texture2D::WRAP_R, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
                        materials[current].specular_map = tex;
                    }
                }
                else if (left(line, 7) == "map_Ke ")
                {
                    const string filename = makePathRelativeTo(line.substr(7), extractPath(libname));
                    Image *image = osgDB::readImageFile(filename);
                    if (image)
                    {
                        Texture2D *tex = new Texture2D(image);
                        tex->setWrap(Texture2D::WRAP_R, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
                        materials[current].emission_map = tex;
                    }
                }
                else if (left(line, 9) == "map_Bump ")
                {
                    const string filename = makePathRelativeTo(line.substr(9), extractPath(libname));
                    Image *image = osgDB::readImageFile(filename);
                    if (image)
                    {
                        Texture2D *tex = new Texture2D(image);
                        tex->setWrap(Texture2D::WRAP_R, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
                        tex->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
                        materials[current].normal_map = tex;
                    }
                }
                break;
            case 's':
                if (parseFormat(line, "sharpness %f", r))
                {
                    materials[current].sharpness = r;
                }
                break;
            }
        }
    }
}
