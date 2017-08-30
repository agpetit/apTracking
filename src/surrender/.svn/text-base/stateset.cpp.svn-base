#include <GL/glew.h>
#include "stateset.h"
#include <iostream>
#include "logs.h"

using namespace std;
using namespace osg;

namespace luxifer
{
    StateSet::StateSet()
    {
        for(size_t i = 0 ; i < (sizeof(hTex) / sizeof(hTex[0])) ; ++i)
            hTex[i] = make_pair(StateAttribute::OFF, make_pair(GL_TEXTURE_2D, 0));
        hProgram = make_pair(osg::StateAttribute::OFF, 0);
        material.second.diffuse = osg::Vec4(1,1,1,1);
        material.second.specular = osg::Vec4(0,0,0,0);
        material.second.emission = osg::Vec4(0,0,0,0);
        material.second.shininess = 1;
        material.first = StateAttribute::ON;
    }

    StateSet::StateSet(const StateSet &s)
    {
        *this = s;
    }

    StateSet &StateSet::operator=(const StateSet &s)
    {
        for(size_t i = 0 ; i < (sizeof(hTex) / sizeof(hTex[0])) ; ++i)
            hTex[i] = s.hTex[i];
        hProgram = s.hProgram;
        uniforms = s.uniforms;
        material = s.material;
        return *this;
    }

    void StateSet::apply()
    {
        CHECK_GL();

        for(size_t i = 0 ; i < (sizeof(hTex) / sizeof(hTex[0])) ; ++i)
        {
            glActiveTexture(GL_TEXTURE0 + i);
            CHECK_GL();
            if (!hTex[i].second.second)
            {
                glDisable(GL_TEXTURE_2D);
                CHECK_GL();
                glDisable(GL_TEXTURE_CUBE_MAP);
                CHECK_GL();
                continue;
            }
            switch(hTex[i].second.first)
            {
            case GL_TEXTURE_2D:
                glDisable(GL_TEXTURE_CUBE_MAP);
                CHECK_GL();
                break;
            case GL_TEXTURE_CUBE_MAP:
                glDisable(GL_TEXTURE_2D);
                CHECK_GL();
                break;
            }
            glEnable(hTex[i].second.first);
            CHECK_GL();
            glBindTexture(hTex[i].second.first, hTex[i].second.second);
            CHECK_GL();
        }

        glMaterialfv(GL_FRONT, GL_DIFFUSE, (const GLfloat*)&material.second.diffuse);
        CHECK_GL();
        glMaterialfv(GL_FRONT, GL_SPECULAR, (const GLfloat*)&material.second.specular);
        CHECK_GL();
        glMaterialfv(GL_FRONT, GL_EMISSION, (const GLfloat*)&material.second.emission);
        CHECK_GL();
        glMaterialf(GL_FRONT, GL_SHININESS, material.second.shininess);
        CHECK_GL();

        if (hProgram.second)
        {
            glUseProgram(hProgram.second);
            CHECK_GL();
            for(uniform_table_t::const_iterator it = uniforms.begin(), end = uniforms.end() ; it != end ; ++it)
            {
                const GLint location = glGetUniformLocation(hProgram.second, it->first.c_str());
                CHECK_GL();
                if (location == -1)
                    continue;
                const osg::ref_ptr<osg::Uniform> &uniform = it->second.second;
                const GLsizei num = uniform->getNumElements();
                switch(osg::Uniform::getGlApiType(uniform->getType()))
                {
                case GL_FLOAT:
                    glUniform1fv(location, num, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_VEC2:
                    glUniform2fv(location, num, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_VEC3:
                    glUniform3fv(location, num, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_VEC4:
                    glUniform4fv(location, num, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_INT:
                    glUniform1iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_INT_VEC2:
                    glUniform2iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_INT_VEC3:
                    glUniform3iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_INT_VEC4:
                    glUniform4iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_BOOL:
                    glUniform1iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_BOOL_VEC2:
                    glUniform2iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_BOOL_VEC3:
                    glUniform3iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_BOOL_VEC4:
                    glUniform4iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_MAT2:
                    glUniformMatrix2fv(location, num, 0, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_MAT3:
                    glUniformMatrix3fv(location, num, 0, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_FLOAT_MAT4:
                    glUniformMatrix4fv(location, num, 0, &(uniform->getFloatArray()->front()));
                    CHECK_GL();
                    break;
                case GL_SAMPLER_1D:
                case GL_SAMPLER_2D:
                case GL_SAMPLER_3D:
                case GL_SAMPLER_CUBE:
                case GL_SAMPLER_1D_SHADOW:
                case GL_SAMPLER_2D_SHADOW:
                    glUniform1iv(location, num, &(uniform->getIntArray()->front()));
                    CHECK_GL();
                    break;

                case GL_SAMPLER_1D_ARRAY_EXT:
                case GL_SAMPLER_2D_ARRAY_EXT:
                case GL_SAMPLER_1D_ARRAY_SHADOW_EXT:
                case GL_SAMPLER_2D_ARRAY_SHADOW_EXT:
                    break;
                }
            }
        }
        else
            glUseProgram(0);
    }

    template<class T>
    inline void setAttribute(pair<StateSet::Mode, T> &out, const T &val, StateSet::Mode mode)
    {
        if (mode & StateAttribute::PROTECTED)
        {
            out.first = mode;
            out.second = val;
        }
        else if (mode & osg::StateAttribute::OVERRIDE)
        {
            if (!(out.first & osg::StateAttribute::OVERRIDE))
            {
                out.first = mode;
                out.second = val;
            }
        }
        else if (mode & osg::StateAttribute::INHERIT)
        {
        }
        else if (!(out.first & osg::StateAttribute::OVERRIDE))
        {
            out.first = mode;
            out.second = val;
        }
    }

    void StateSet::setTexture(size_t idx, GLuint target, GLuint htex, Mode mode)
    {
        setAttribute(hTex[idx], make_pair(target, htex), mode);
    }

    void StateSet::setUniforms(const osg::StateSet::UniformList &uniforms)
    {
        for(osg::StateSet::UniformList::const_iterator it = uniforms.begin() ; it != uniforms.end() ; ++it)
        {
            const int override = it->second.second;
            osg::ref_ptr<osg::Uniform> uniform = it->second.first;
            const std::string &name = uniform->getName();
            uniform_table_t::iterator u_it = this->uniforms.find(name);
            if (override & osg::StateAttribute::PROTECTED)
            {
                if (u_it != this->uniforms.end())
                {
                    u_it->second.first = false;
                    u_it->second.second = uniform;
                }
                else
                    this->uniforms[name] = std::make_pair(false, uniform);
            }
            else if (override & osg::StateAttribute::OVERRIDE)
            {
                if (u_it == this->uniforms.end())
                {
                    this->uniforms[name] = std::make_pair(true, uniform);
                }
                else if (!u_it->second.first)
                {
                    u_it->second = std::make_pair(true, uniform);
                }
            }
            else if (override & osg::StateAttribute::INHERIT)
            {
            }
            else if (u_it == this->uniforms.end())
            {
                this->uniforms[name] = std::make_pair(false, uniform);
            }
            else if (!u_it->second.first)
                u_it->second.second = uniform;
        }
    }

    void StateSet::setProgram(GLuint program, Mode mode)
    {
        setAttribute(hProgram, program, mode);
    }

    void StateSet::setMaterial(ref_ptr<const osg::Material> mtl, Mode mode)
    {
        Material mat;
        mat.diffuse = mtl->getDiffuse(osg::Material::FRONT);
        mat.specular = mtl->getSpecular(osg::Material::FRONT);
        mat.emission = mtl->getEmission(osg::Material::FRONT);
        mat.shininess = mtl->getShininess(osg::Material::FRONT);
        setAttribute(material, mat, mode);
    }
}
