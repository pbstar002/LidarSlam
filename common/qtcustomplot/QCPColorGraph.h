#include <QVector>
#include <QColor>
#include "qcustomplot.h"

class QCPColorGraph : public QCPGraph
{
public:
    QCPColorGraph(QCPAxis *keyAxis, QCPAxis *valueAxis);
    virtual ~QCPColorGraph();
    void setData(const QVector<double> &keys, const QVector<double> &values, const QVector<QColor> &colors);

protected:
    virtual void drawScatterPlot(QCPPainter *painter, const QVector<QPointF> &points, const QCPScatterStyle &style) const;

private:
    QVector<QColor> colors_;
};