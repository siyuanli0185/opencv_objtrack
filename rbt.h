#ifndef RBT_H
#define RBT_H

#include <opencv\cv.h>

using namespace cv;

class rbt
{
	Point2i head, body;
public:
	rbt();
	rbt(Point2i pt_head, Point2i pt_body);
	void setHead(int x, int y);
	void setHead(Point2i pt_head) {head = pt_head;}
	void setBody(int x, int y);
	void setBody(Point2i pt_body) {body = pt_body;}
	Point2i getHead() {return head;}
	Point2i getBody() {return body;}
	bool rbt_available();
};

rbt::rbt()
{
	head.x = -1; 
	head.y = -1;
	body.x = -1;
	body.y = -1;
}

rbt::rbt(Point2i pt_head, Point2i pt_body) 
{
	head = pt_head;
	body = pt_body;
}

void rbt::setHead(int x, int y)
{
	head.x = x;
	head.y = y;
}

void rbt::setBody(int x, int y)
{
	body.x = x;
	body.y = y;
}

bool rbt::rbt_available()
{
	if (head.x != -1 && body.x != -1)
		return true;
	else return false;
}

#endif
