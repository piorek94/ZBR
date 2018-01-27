#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "kinematics.h"
#include "trajectorydialog.h"
#include "infodialog.h"

#include "qcustomplot.h"                                    //plotterdialog
#include "typedefs.h"                                       //plotterdialog
#include <QDialog>                                          //plotterdialog
#include <QSize>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    static double radToDeg(double rad);
    static double degToRad(double deg);
    void setPlotData1(QList<machineCoordinates>* coords);   //plotterdialog
    void setBasicQPlot();                                   //plotterdialog

private:
    Ui::MainWindow *ui;
    Kinematics kinematics;
    Trajectory *trajectory;

    QVector<double>* fi1;                                   //plotterdialog
    QVector<double>* fi2;                                   //plotterdialog
    QVector<double>* fi3;                                   //plotterdialog
    QVector<double>* fi4;                                   //plotterdialog
    QVector<double>* fi5;                                   //plotterdialog
    QVector<double>* time;                                  //plotterdialog

    approachVector aV;
    machineCoordinates mC;
    robotParamsRegional rPRegional;
    robotParamsLocal rPLocal;
    Deltas deltas;

    point3D tcpStart, tcpEnd;
    int trajectoryPointNumber;

    QList<machineCoordinates>* resultCoordinates;

    QMessageBox *wrongTrajectoryMessage;
    TrajectoryDialog *trajectoryDialog;
    InfoDialog *infoDialog;

    QPalette palette;
    QPainter *painterXY;
    QPainter *painterYZ;
    QPainter *painterXZ;
    QPixmap *pixmapXY;
    QPixmap *pixmapYZ;
    QPixmap *pixmapXZ;

    float factor;

    QTimer *timer;

    void updateKinematics();
    void updateTrajectory();
    void updateStartTCP();
    void updateEndTCP();
    point3D getMaxCoords();

    void paintXY(int i);
    void paintYZ(int i);
    void paintXZ(int i);

private slots:
    void on_updateButton_clicked();
    void on_calcTrajectoryButton_clicked();
    void on_trajectoryEditFinished(Trajectory* trajectory);
    void on_timer_timeout();

    void on_horizontalSlider_valueChanged(int value);
    void on_animationButton_clicked();
    void on_infoButton_clicked();
};

#endif // MAINWINDOW_H
