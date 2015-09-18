#ifndef LINE_H
#define LINE_H

#include <Point.h>

namespace katana
{

class Line
{
  public:
    Line(Point startPoint, Point endPoint)	{m_startPoint = startPoint; m_endPoint = endPoint; }
    virtual ~Line()  {}

    //! Getter
    virtual Point getStartPoint() const         { return m_startPoint; }
    virtual Point getEndPoint() const           { return m_endPoint; }

    //! Setter
    virtual void setStartPoint(Point p)         { m_startPoint = p; }
    virtual void setEndPoint(Point p)           { m_endPoint = p; }

    //! Access via reference
    virtual Point& startPunkt()                 { return m_startPoint; }
    virtual Point& endPoint()                   { return m_endPoint; }

    double getLenght()				{
      _position_type x = getXdiff();
      _position_type y = getYdiff();

      return sqrt(pow(x,2)+pow(y,2));
    }

    double_t operator*(Line& rhs)			{
      _position_type x1 = getXdiff();
      _position_type x2 = rhs.getXdiff();
      _position_type y1 = getYdiff();
      _position_type y2 = rhs.getYdiff();

      return x1*x2 + y1*y2;
    }

    Point getPointOnLine(tFloat32 splitFactor);

  protected:
    _position_type getXdiff() 				{ return m_startPoint.getX() - m_endPoint.getX(); }
    _position_type getYdiff()				{ return m_startPoint.getY() - m_endPoint.getY(); }

  private:
    Point m_startPoint;
    Point m_endPoint;
};

}

#endif // LINE_H
