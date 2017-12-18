#ifndef STRUCTURES_H
#define STRUCTURES_H


typedef struct point3d{

  double x;
  double y;
  double z;
}point3d;

typedef struct point2d{

  int i;
  int j;
}point2d;

typedef struct triangle{

  int v1;
  int n1;

  int v2;
  int n2;

  int v3;
  int n3;

}triangle;


#endif
