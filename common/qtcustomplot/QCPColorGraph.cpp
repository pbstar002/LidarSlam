#include "QCPColorGraph.h"
 
QCPColorGraph::QCPColorGraph(QCPAxis *keyAxis, QCPAxis *valueAxis) : QCPGraph(keyAxis, valueAxis) {}
 
QCPColorGraph::~QCPColorGraph(){ }
 
void QCPColorGraph::setData(const QVector<double> & keys, const QVector<double> & values, const QVector<QColor> & colors){
    if (values.size() != colors.size()) return;
    colors_ = colors;
    QCPGraph::setData(keys, values);
}
 
void QCPColorGraph::drawScatterPlot(QCPPainter * painter, const QVector<QPointF> & points, const QCPScatterStyle & style) const {
    applyScattersAntialiasingHint(painter);
    int nPoints = points.size();
    for (int i = 0; i < nPoints; ++i)
        if (!qIsNaN(points.at(i).x()) && !qIsNaN(points.at(i).y())){
            painter->setPen(colors_[i]);
            style.drawShape(painter, points.at(i));
        }
}