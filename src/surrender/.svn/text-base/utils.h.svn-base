#ifndef UTILS_H
#define UTILS_H

#include <osg/Object>
#include <iostream>
#include <string>
#include <vector>
#include <osg/BoundingSphere>

namespace luxifer
{

    /** \brief Print the structure of the given node (childs, name, type, ...)
      *
      * The purpose of this function is to produce a human-readable description
      * of a scene graph.
      */
    void printGraphStructure(const osg::Object *node, std::ostream &out = std::cout, const std::string &prefix = std::string());

    /** \brief Returns all objects with the given name inside a graph
      *
      * \param node the graph to search for objects with given name
      * \param name the name of the objects we want
      * \param out a vector to fill with the result
      */
    void getObjectsByName(const osg::Object *node, const std::string &name, std::vector<const osg::Object*> &out);

    //! \brief Returns the size in bytes of the scene graph
    size_t sizeOfGraphStructure(const osg::Object *node);

    /** \brief Returns the bounding sphere of the scene graph
      *
      * osg::Node::getBound precision is not reliable (depending on build options
      * it may use float variables which is not precise enough for us) whereas this
      * function guaranties double precision for everything except vertex data itself.
      */
    osg::BoundingSphered computeBoundsOf(const osg::Object *node);

    //! \brief Returns a lower case copy of str
    std::string lower(const std::string &str);
    //! \brief Returns an upper case copy of str
    std::string upper(const std::string &str);
    //! \brief Returns the first len chars of str
    std::string left(const std::string &str, const size_t len);
    //! \brief Returns the last len chars of str
    std::string right(const std::string &str, const size_t len);
    //! \brief Returns a copy of str with given characters escaped
    std::string escape(const std::string &str, const std::string &chars = "\"\\", const char escape_char = '\\');

    //! \brief Reads an int from a std::string
    int parseInt(const char *str, size_t *len);
    //! \brief Reads a float from a std::string (understands both '.' and ',')
    float parseFloat(const char *str, size_t *len);
    //! \brief Kind of simplified scanf, locale independent
    int parseFormat(const char *str, const std::string &fmt, void **out);
    template<class T>
    inline int parseFormat(const char *str, const std::string &fmt, T &o0)
    {
        void *out[] = { &o0 };
        return parseFormat(str, fmt, out);
    }
    template<class T>
    inline int parseFormat(const std::string &str, const std::string &fmt, T &o0)
    {   return parseFormat(str.c_str(), fmt, o0);   }
    template<class T, class U>
    inline int parseFormat(const char *str, const std::string &fmt, T &o0, U &o1)
    {
        void *out[] = { &o0, &o1 };
        return parseFormat(str, fmt, out);
    }
    template<class T, class U>
    inline int parseFormat(const std::string &str, const std::string &fmt, T &o0, U &o1)
    {   return parseFormat(str.c_str(), fmt, o0, o1);   }
    template<class T, class U, class V>
    inline int parseFormat(const char *str, const std::string &fmt, T &o0, U &o1, V &o2)
    {
        void *out[] = { &o0, &o1, &o2 };
        return parseFormat(str, fmt, out);
    }
    template<class T, class U, class V>
    inline int parseFormat(const std::string &str, const std::string &fmt, T &o0, U &o1, V &o2)
    {   return parseFormat(str.c_str(), fmt, o0, o1, o2);   }
    template<class T, class U, class V, class X>
    inline int parseFormat(const char *str, const std::string &fmt, T &o0, U &o1, V &o2, X &o3)
    {
        void *out[] = { &o0, &o1, &o2, &o3 };
        return parseFormat(str, fmt, out);
    }
    template<class T, class U, class V, class X>
    inline int parseFormat(const std::string &str, const std::string &fmt, T &o0, U &o1, V &o2, X &o3)
    {   return parseFormat(str.c_str(), fmt, o0, o1, o2, o3);   }

    /** \brief Similar to std::pair but with 3 elements
      *
      * It also implements lexicographical order (operator<)
      */
    template<typename T, typename U, typename V>
    class tuple
    {
    public:
        tuple(const T &t, const U &u, const V &v) : first(t), second(u), third(v)   {}
        tuple(const tuple &t) : first(t.first), second(t.second), third(t.third)   {}

        tuple &operator=(const tuple &t)
        {
            first = t.first;
            second = t.second;
            third = t.third;
            return *this;
        }

        bool operator<(const tuple &t) const
        {
            return first < t.first
                    || (first == t.first
                        && (second < t.second
                            || (second == t.second
                                && third < t.third)));
        }

        bool operator==(const tuple &t) const
        {
            return first == t.first
                    && second == t.second
                    && third == t.third;
        }

    public:
        T first;
        U second;
        V third;
    };

    template<typename T, typename U, typename V>
    inline tuple<T,U,V> make_tuple(const T &t, const U &u, const V &v)
    {
        return tuple<T,U,V>(t,u,v);
    }

    /** \brief Interprets the given path as a path relative to ref_path if not an absolute path
      *
      * C:\\absolute_path -> C:\\absolute_path
      *
      * /absolute/path -> /absolute/path
      *
      * relative/path -> ref_path/relative/path
      */
    std::string makePathRelativeTo(const std::string &path, const std::string &ref_path);

    /** \brief Returns the path to the parent folder
      *
      * For a file it returns the folder which contains the file.
      * For a folder it returns its parent.
      */
    std::string extractPath(const std::string &path);

    /** \brief Functor interface
      *
      * This interface is mainly used to implement a generic LiDAR raytracer
      * without templating it.
      */
    class Function
    {
    public:
        virtual double operator()(double) const = 0;
    };
}

#endif
