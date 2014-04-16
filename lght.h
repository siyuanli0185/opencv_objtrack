#ifndef LGHT_H
#define LGHT_H

#include <opencv\cv.h>

using namespace cv;
const int maxSize = 3;

class lght
{
	Point2i light[maxSize];
	unsigned int index;
public:
	lght();
	lght(Point2i position);
	lght(int x, int y);
	void addLgt(Point2i position);
	void addLgt(int x, int y);
	Point2i getLgt(unsigned int n);
	unsigned int getIndex();
	bool lght_available();
};

lght::lght()
{
	Point2i deflt(-1,-1);
	for (int i = 0; i < maxSize; i++)
	{
		light[i] = deflt;
	}
	index = 0;
}

lght::lght(Point2i position) 
{
	light[0] = position;
	index = 1;
}

lght::lght(int x, int y) 
{
	light[0].x = x;
	light[0].y = y;
	index = 1;
}

void lght::addLgt(Point2i position)
{
	if (index == 0)
		light[index] = position;
	else if (index < maxSize)
	{
		light[index] = position;
		index++;
	}
	else
	{
		std::cout << "Light array is full" << std::endl;
	}
}

void lght::addLgt(int x, int y)
{
	if (index == 0)
	{
		light[index].x = x;
		light[index].y = y;
		index++;
	}
	else if (index < maxSize)
	{
		light[index].x = x;
		light[index].y = y;
		index++;
	}
	else
	{
		std::cout << "Light array is full" << std::endl;
	}

}

Point2i lght::getLgt(unsigned int n)
{
	return light[n];
}

unsigned int lght::getIndex()
{
	return index;
}

bool lght::lght_available()
{
	for(unsigned int i = 0; i <= index; i++)
	{
		if (light[i].x != -1 && light[i].y != -1)
			return true;
	}
	return false;
}

#endif
