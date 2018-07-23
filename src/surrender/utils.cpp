#if defined(WIN32)
#include <algorithm>
#endif

#include "utils.h"
#include <osg/Camera>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/LightSource>
#include <osg/PrimitiveSet>
#include <osg/Array>
#include <locale>

using namespace std;

namespace luxifer
{
    void printGraphStructure(const osg::Object *node, ostream &out, const string &prefix)
    {
        if (!node)
        {
            out << prefix << "[null]";
            if (prefix.empty())
                out << endl;
            return;
        }
        const string object_name = node->getName() + '(' + node->libraryName() + "::" + node->className() + ')';
        out << object_name;
        if (dynamic_cast<const osg::Camera*>(node))
        {
            if (prefix.empty())
                out << endl;
            return;
        }

        const osg::Group* group = dynamic_cast<const osg::Group*>(node);
        if (group)
        {
            for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
            {
                string new_prefix;
                if (i == 0)
                {
                    new_prefix = prefix + string(object_name.size(),' ');
                    if (nb == 1)
                        new_prefix += "    ";
                    else
                        new_prefix += "|   ";
                    out << "--> ";
                }
                else if (i + 1 < nb)
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "|   ";
                    out << prefix << string(object_name.size(),' ') << "|-> ";
                }
                else
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "    ";
                    out << prefix << string(object_name.size(),' ') << "\\-> ";
                }
                printGraphStructure(group->getChild(i), out, new_prefix);
                if (i + 1 < nb)
                    out << endl;
            }
        }

        const osg::Geode *geode = dynamic_cast<const osg::Geode*>(node);
        if (geode)
        {
            for(unsigned int i = 0, nb = geode->getNumDrawables() ; i < nb ; ++i)
            {
                string new_prefix;
                if (i == 0)
                {
                    new_prefix = prefix + string(object_name.size(),' ');
                    if (nb == 1)
                        new_prefix += "    ";
                    else
                        new_prefix += "|   ";
                    out << "--> ";
                }
                else if (i + 1 < nb)
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "|   ";
                    out << prefix << string(object_name.size(),' ') << "|-> ";
                }
                else
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "    ";
                    out << prefix << string(object_name.size(),' ') << "\\-> ";
                }
                printGraphStructure(geode->getDrawable(i), out, new_prefix);
                if (i + 1 < nb)
                    out << endl;
            }
        }

        const osg::Geometry *geom = dynamic_cast<const osg::Geometry*>(node);
        if (geom)
        {
            for(unsigned int i = 0, nb = geom->getNumPrimitiveSets() ; i < nb ; ++i)
            {
                string new_prefix;
                if (i == 0)
                {
                    new_prefix = prefix + string(object_name.size(),' ');
                    if (nb == 1)
                        new_prefix += "    ";
                    else
                        new_prefix += "|   ";
                    out << "--> ";
                }
                else if (i + 1 < nb)
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "|   ";
                    out << prefix << string(object_name.size(),' ') << "|-> ";
                }
                else
                {
                    new_prefix = prefix + string(object_name.size(),' ') + "    ";
                    out << prefix << string(object_name.size(),' ') << "\\-> ";
                }
                printGraphStructure(geom->getPrimitiveSet(i), out, new_prefix);
                if (i + 1 < nb)
                    out << endl;
            }
        }

        if (prefix.empty())
            out << endl;
    }

    void getObjectsByName(const osg::Object *node, const std::string &name, vector<const osg::Object*> &out)
    {
        if (!node)
            return;
        if (node->getName() == name)
            out.push_back(node);
        if (dynamic_cast<const osg::Camera*>(node))
            return;

        const osg::Group* group = dynamic_cast<const osg::Group*>(node);
        if (group)
        {
            for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
                getObjectsByName(group->getChild(i), name, out);
        }

        const osg::Geode *geode = dynamic_cast<const osg::Geode*>(node);
        if (geode)
        {
            for(unsigned int i = 0, nb = geode->getNumDrawables() ; i < nb ; ++i)
                getObjectsByName(geode->getDrawable(i), name, out);
        }

        const osg::Geometry *geom = dynamic_cast<const osg::Geometry*>(node);
        if (geom)
        {
            for(unsigned int i = 0, nb = geom->getNumPrimitiveSets() ; i < nb ; ++i)
                getObjectsByName(geom->getPrimitiveSet(i), name, out);
        }
    }

    size_t sizeOfGraphStructure(const osg::Object *node)
    {
        if (!node)
            return 0;
        if (dynamic_cast<const osg::Camera*>(node))
            return sizeof(osg::Camera);

        size_t s = 0;
        if (dynamic_cast<const osg::PositionAttitudeTransform*>(node))
            s += sizeof(osg::PositionAttitudeTransform) - sizeof(osg::Group);
        if (dynamic_cast<const osg::MatrixTransform*>(node))
            s += sizeof(osg::MatrixTransform) - sizeof(osg::Group);
        if (dynamic_cast<const osg::LightSource*>(node))
            s += sizeof(osg::LightSource) - sizeof(osg::Group);

        const osg::Group* group = dynamic_cast<const osg::Group*>(node);
        if (group)
        {
            s += sizeof(osg::Group);
            for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
                s += sizeOfGraphStructure(group->getChild(i));
            return s;
        }

        const osg::Geode *geode = dynamic_cast<const osg::Geode*>(node);
        if (geode)
        {
            s += sizeof(osg::Geode);
            for(unsigned int i = 0, nb = geode->getNumDrawables() ; i < nb ; ++i)
                s += sizeOfGraphStructure(geode->getDrawable(i));
            return s;
        }

        const osg::Geometry *geom = dynamic_cast<const osg::Geometry*>(node);
        if (geom)
        {
            s += sizeof(osg::Geometry);
            if (geom->getVertexArray())
                s += geom->getVertexArray()->getDataSize() * geom->getVertexArray()->getNumElements();
            if (geom->getNormalArray())
                s += geom->getNormalArray()->getDataSize() * geom->getNormalArray()->getNumElements();
            if (geom->getTexCoordArray(0))
                s += geom->getTexCoordArray(0)->getDataSize() * geom->getTexCoordArray(0)->getNumElements();
            for(unsigned int i = 0, nb = geom->getNumPrimitiveSets() ; i < nb ; ++i)
                s += sizeOfGraphStructure(geom->getPrimitiveSet(i));
            return s;
        }

        const osg::DrawElements *prim = dynamic_cast<const osg::DrawElements*>(node);
        if (prim)
        {
            s += sizeof(osg::DrawElements);
            s += prim->getTotalDataSize();
            return s;
        }

        if (dynamic_cast<const osg::Node*>(node))
            return sizeof(osg::Node);
        return sizeof(osg::Object);
    }

    osg::BoundingSphered computeBoundsOf(const osg::Object *node)
    {
        if (!node || dynamic_cast<const osg::Camera*>(node))
            return osg::BoundingSphered();

        const osg::Group* group = dynamic_cast<const osg::Group*>(node);
        if (group)
        {
            osg::BoundingSphered bound;
            for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
                bound.expandBy(computeBoundsOf(group->getChild(i)));

            const osg::PositionAttitudeTransform * const pat = dynamic_cast<const osg::PositionAttitudeTransform*>(node);
            if (pat)
            {
                const osg::Matrixd &mat = osg::Matrixd::translate(-pat->getPivotPoint())
                                        * osg::Matrixd::rotate(pat->getAttitude())
                                        * osg::Matrixd::scale(pat->getScale())
                                        * osg::Matrixd::translate(pat->getPosition());
                bound.radius() *= std::max(pat->getScale().x(),
                                      std::max(pat->getScale().y(),
                                          pat->getScale().z()));
                bound.center() = bound.center() * mat;
            }
            else
            {
                const osg::MatrixTransform * const mt = dynamic_cast<const osg::MatrixTransform*>(node);
                if (mt)
                {
                    const osg::Matrixd mat = mt->getMatrix();
                    const osg::Vec3d &s = mat.getScale();
                    bound.radius() *= std::max(s.x(), std::max(s.y(), s.z()));
                    bound.center() = bound.center() * mat;
                }
            }
            return bound;
        }

        const osg::Geode *geode = dynamic_cast<const osg::Geode*>(node);
        if (geode)
        {
            osg::BoundingSphered bound;
            for(unsigned int i = 0, nb = geode->getNumDrawables() ; i < nb ; ++i)
                bound.expandBy(computeBoundsOf(geode->getDrawable(i)));
            return bound;
        }

        const osg::Geometry *geom = dynamic_cast<const osg::Geometry*>(node);
        if (geom)
        {
            const osg::BoundingSphere &bound = geom->getBound();
            return osg::BoundingSphered(bound.center(), bound.radius());
        }

        return osg::BoundingSphered();
    }

    string lower(const string &str)
    {
        const locale loc;
        string ret;
        ret.reserve(str.size());
        for(size_t i = 0 ; i < str.size() ; ++i)
            ret.push_back(tolower(str[i], loc));
        return ret;
    }

    string upper(const string &str)
    {
        const locale loc;
        string ret;
        ret.reserve(str.size());
        for(size_t i = 0 ; i < str.size() ; ++i)
            ret.push_back(toupper(str[i], loc));
        return ret;
    }

    string left(const string &str, const size_t len)
    {
        return str.substr(0, len);
    }

    string right(const string &str, const size_t len)
    {
        return str.substr(str.size() - len, len);
    }

    inline bool isdigit(char c)
    {
        return c >= '0' && c <= '9';
    }

    int parseInt(const char *str, size_t *len)
    {
        int v(0);
        size_t p = 0;
        bool b_negative = false;
        if (str[0] == '-')
        {
            b_negative = true;
            p = 1;
        }
        for( ; isdigit(str[p]) ; ++p)
            v = v * 10 + (str[p] - '0');
        if (b_negative)
            v = -v;
        if (len)
            *len = p;
        return v;
    }

    float parseFloat(const char *str, size_t *len)
    {
        double v(0.0);
        size_t p = 0;
        bool b_negative = false;
        if (str[0] == '-')
        {
            b_negative = true;
            p = 1;
        }
        for( ; isdigit(str[p]) ; ++p)
            v = v * 10.0 + (str[p] - '0');

        if (str[p] == '.' || str[p] == ',')
        {
            ++p;
            for(double f = 0.1 ; isdigit(str[p]) ; ++p, f *= 0.1)
                v += (str[p] - '0') * f;
        }

        if (str[p] == 'e' || str[p] == 'E')
        {
            size_t l;
            ++p;
            const int e = parseInt(str + p, &l);
            p += l;
            v *= pow(10.0, e);
        }

        if (b_negative)
            v = -v;
        if (len)
            *len = p;
        return v;
    }

    int parseFormat(const char *str, const std::string &fmt, void *out[4])
    {
        size_t p = 0;
        int n = 0;
        for(size_t i = 0 ; i < fmt.size() ; ++i)
        {
            switch(fmt[i])
            {
            case '%':
                ++i;
                if (i == fmt.size())
                    return n;
                switch(fmt[i])
                {
                case 'd':
                    {
                        size_t l;
                        *((int*)out[n]) = parseInt(str + p, &l);
                        if (l == 0)
                            return n;
                        p += l;
                    }
                    break;
                case 'f':
                    {
                        size_t l;
                        *((float*)out[n]) = parseFloat(str + p, &l);
                        if (l == 0)
                            return n;
                        p += l;
                    }
                    break;
                default:
                    return n;
                }

                ++n;
                break;
            default:
                if (fmt[i] != str[p])
                    return n;
                ++p;
            }
        }
        return n;
    }

    std::string makePathRelativeTo(const std::string &path, const std::string &ref_path)
    {
        if (path.empty())
            return ref_path;

        // Unix absolute path
        if (path[0] == '/')
            return path;

        // Windows absolute path
        if (path.size() > 1 && path[2] == ':')
            return path;

        if (ref_path.empty())
            return path;

        if (*ref_path.rbegin() == '/' || *ref_path.rbegin() == '\\')
            return ref_path + path;
        return ref_path + '/' + path;
    }

    string extractPath(const string &path)
    {
        string::size_type p = path.find_last_of("/\\");
        if (p != string::npos)
        {
            const string ret = left(path, p);
            if (ret.empty() && !path.empty())
                return "/";
            if (ret.size() == 2 && ret[1] == ':')
                return ret + '/';
            return ret;
        }
        return string();
    }

    string escape(const string &str, const string &chars, const char escape_char)
    {
        string ret;
        for(size_t i = 0 ; i < str.size() ; ++i)
        {
            if (chars.find(str[i]) != string::npos)
                ret += escape_char;
            ret += str[i];
        }
        return ret;
    }
}
