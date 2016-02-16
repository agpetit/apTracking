#pragma once
#include <iostream>
#include <vector>

using namespace std;

template <class T>
class BOMatrix
{
public:
	int r, c;
	BOMatrix(){};
	~BOMatrix(){};
	BOMatrix(int rows, int cols):data(rows, vector<T>(cols)){r = rows; c= cols;}
	inline vector<T> & operator[](int i) { return data[i];}
	inline const vector<T> & operator[] (int i) const { return data[i];}

	friend  BOMatrix<T>& operator *= ( BOMatrix<T> &m, const T& v)
	{
		for(int i=0; i<m.data.size(); i++)
			for(int j=0; j<m.data[i].size(); j++)
				m.data[i][j] *=  v;
			return m;
	}
	friend  BOMatrix<T>& operator /= ( BOMatrix<T> &m, const T& v)
	{
		for(int i=0; i<m.data.size(); i++)
			for(int j=0; j<m.data[i].size(); j++)
				m.data[i][j] /=  v;
			return m;
	}

	int sizeR(){ return data.size(); }
	int sizeC(){ if(data.size()>0) return data[0].size();	else return 0; }
	BOMatrix<T> Example( );

public:
	vector<vector<T> > data;  
};
