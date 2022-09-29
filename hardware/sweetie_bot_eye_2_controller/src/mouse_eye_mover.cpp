#include "mouse_eye_mover.h"

MouseEyeMoverScene::MouseEyeMoverScene(QObject *parent)
  :QGraphicsScene()
{
  rVert.setRect(-CROSS_WIDTH,-CROSS_LENGTH,CROSS_WIDTH*2,CROSS_LENGTH*2);
  rVert.setPen(QPen(Qt::black));
  rVert.setBrush(QBrush(Qt::green));
  rVert.setPos(EYE_VIEW_WIDTH/2, EYE_VIEW_WIDTH/2);

  rGoriz.setRect(-CROSS_LENGTH,-CROSS_WIDTH,CROSS_LENGTH*2,CROSS_WIDTH*2);
  rGoriz.setPen(QPen(Qt::black));
  rGoriz.setBrush(QBrush(Qt::green));
  rGoriz.setPos(EYE_VIEW_WIDTH/2, EYE_VIEW_WIDTH/2);
  rGoriz.setZValue(1);
  rVert.setZValue(1);
  this->addItem(&rVert);
  this->addItem(&rGoriz);
}
void MouseEyeMoverScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mousePressEvent(event);

  if (!event->isAccepted())
  {
    if (event->button() == Qt::LeftButton)
    {
      QPointF pos = event->scenePos();
      if (pos.x()>EYE_VIEW_WIDTH) pos.setX(EYE_VIEW_WIDTH);
      if (pos.x()<0)  pos.setX(0);
      if (pos.y()>EYE_VIEW_WIDTH) pos.setY(EYE_VIEW_WIDTH);
      if (pos.y()<0) pos.setY(0);
      rVert.setPos(pos);
      rGoriz.setPos(pos);
      RecalcAndSend(pos);
    }
  }
}

void MouseEyeMoverScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mouseMoveEvent(event);

  QPointF pos = event->scenePos();
  if (pos.x()>EYE_VIEW_WIDTH) pos.setX(EYE_VIEW_WIDTH);
  if (pos.x()<0)  pos.setX(0);
  if (pos.y()>EYE_VIEW_WIDTH) pos.setY(EYE_VIEW_WIDTH);
  if (pos.y()<0) pos.setY(0);
  rVert.setPos(pos);
  rGoriz.setPos(pos);
  RecalcAndSend(pos);
}

void MouseEyeMoverScene::RecalcAndSend(QPointF p)
{
  p.setX((p.x()-EYE_VIEW_CENTR)*RADIAN_COEF);
  p.setY((p.y()-EYE_VIEW_CENTR)*RADIAN_COEF);
  emit SendPoint(p);
}

