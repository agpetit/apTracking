#ifndef MODELLOADER_H
#define MODELLOADER_H

#include <osg/Node>
#include <osg/Texture2D>
#include <osg/Array>
#include <osg/Geometry>
#include <string>
#include <map>

namespace luxifer
{
    /** \brief An interface which handles loading 3D meshes
      *
      * This interface implements support to efficiently parse OBJ files. For other
      * file format OpenSceneGraph plugins are used.
      *
      * NB: the OBJ parser is designed to avoid fragmenting memory too much and its memory
      * requirements are low compared to OpenSceneGraph plugin for OBJ. Huge OBJ files can
      * be loaded without memory allocation error (over 1M triangles).
      */
    class ModelLoader
    {
        struct Material
        {
            Material();
            Material(const Material &m) {   *this = m;  }

            osg::ref_ptr<osg::Texture2D> diffuse_map;
            osg::ref_ptr<osg::Texture2D> specular_map;
            osg::ref_ptr<osg::Texture2D> normal_map;
            osg::ref_ptr<osg::Texture2D> emission_map;
            osg::Vec4d ambient;
            osg::Vec4d diffuse;
            osg::Vec4d specular;
            osg::Vec4d emissive;
            double sharpness;
            double Ns;          // shininess
        };

    private:
        ModelLoader();

        osg::ref_ptr<osg::Node> loadOBJ(const std::string &filename);
        osg::ref_ptr<osg::Node> loadOFF(const std::string &filename);

        void loadMaterialLib(const std::string &libname);
        void postProcess(osg::ref_ptr<osg::Vec3Array> v_vertex,
                         osg::ref_ptr<osg::Vec3Array> v_normal,
                         osg::ref_ptr<osg::Vec2Array> v_tcoord,
                         osg::ref_ptr<osg::DrawElementsUInt> v_index,
                         const std::vector<bool> &normal_is_null);
        void setMaterial(const std::string &mtlname, osg::ref_ptr<osg::Geometry> geom);
    public:
        static osg::ref_ptr<osg::Node> load(const std::string &filename);

    private:
        std::map<std::string, Material> materials;
    };
}

#endif // MODELLOADER_H
