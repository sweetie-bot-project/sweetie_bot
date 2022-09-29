#ifndef MOUSEEYEMOVER_H
#define MOUSEEYEMOVER_H

#include <QGraphicsObject>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>

#define EYE_VIEW_WIDTH    800


#define EYE_VIEW_CENTR  EYE_VIEW_WIDTH/2
#define RADIAN_COEF     (3.14/EYE_VIEW_WIDTH)

#define CROSS_WIDTH     3
#define CROSS_LENGTH    40


class MouseEyeMoverScene :public QGraphicsScene
{
  Q_OBJECT
public:
  explicit MouseEyeMoverScene(QObject *parent = nullptr);

signals:
  void SendPoint(QPointF p);


private:
  QGraphicsRectItem rVert,rGoriz;
  void RecalcAndSend(QPointF p);
  void mousePressEvent(QGraphicsSceneMouseEvent *event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
};


#endif // MOUSEEYEMOVER_H
