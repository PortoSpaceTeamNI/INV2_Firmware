#ifndef HDC_H
#define HDC_H

#include <HDC302x.h>

int InitializeHDC3020();
int ReadHDC3020(HDC302xDataResult& result);
bool IsHDC3020Ready();

#endif // HDC_H