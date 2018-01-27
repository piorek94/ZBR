#include "mainwindow.h"

#include "ui_mainwindow.h"

#include "trajectory.h"
#include "kinematics.h"
#include "math.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
     trajectory = NULL;
     ui->setupUi(this);
     this->setFixedSize(size());
     this->setWindowTitle("Wizualizacja ruchu TCP robota");
     this->setBasicQPlot();
     ui->horizontalSlider->setEnabled(false);
     ui->animationButton->setEnabled(false);
     wrongTrajectoryMessage = new QMessageBox(this);
     palette.setColor(QPalette::Background, Qt::cyan);
     wrongTrajectoryMessage->setPalette(palette);
     wrongTrajectoryMessage->setIcon(QMessageBox::Critical);
     wrongTrajectoryMessage->setWindowTitle("Błąd");
     wrongTrajectoryMessage->setText("Wybrane punkty są nieosiągalne.");
     wrongTrajectoryMessage->setInformativeText("Nie można zrealizować trajektori. Należy ponownie wprowadzić wartości punktów.");
     wrongTrajectoryMessage->setStandardButtons(QMessageBox::Cancel);
     wrongTrajectoryMessage->setDefaultButton(QMessageBox::Cancel);
     trajectoryDialog = new TrajectoryDialog(this);
     infoDialog = new InfoDialog(this);


     pixmapXY = new QPixmap(ui->XYView->size());
     pixmapYZ = new QPixmap(ui->YZView->size());
     pixmapXZ = new QPixmap(ui->XZView->size());
     pixmapXY->fill(QColor("lightGrey"));
     pixmapYZ->fill(QColor("lightGray"));
     pixmapXZ->fill(QColor("lightGray"));
     painterXY = new QPainter(pixmapXY);
     painterXZ = new QPainter(pixmapXZ);
     painterYZ = new QPainter(pixmapYZ);
     /*OPIS OSI*/
     painterXZ->setPen(QPen(QBrush(Qt::blue), 1));
     //os X pozioma
     painterXZ->drawLine(0,pixmapXZ->height()/2,pixmapXZ->width(),pixmapXZ->height()/2);
     painterXZ->drawLine(0,pixmapXZ->height()/2,10,pixmapXZ->height()/2-7);
     painterXZ->drawLine(0,pixmapXZ->height()/2,10,pixmapXZ->height()/2+7);
     //litera X
     painterXZ->drawLine(5,pixmapXZ->height()/2+8, 15,pixmapXZ->height()/2+18);
     painterXZ->drawLine(5,pixmapXZ->height()/2+18, 15,pixmapXZ->height()/2+8);
     //os Z pionowa
     painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2,pixmapXZ->height());
     painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2-7,10);
     painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2+7,10);
     //litera Z
     painterXZ->drawLine(pixmapXZ->width()/2+8,5,pixmapXZ->width()/2+18,5);
     painterXZ->drawLine(pixmapXZ->width()/2+18,5,pixmapXZ->width()/2+8,15);
     painterXZ->drawLine(pixmapXZ->width()/2+8,15,pixmapXZ->width()/2+18,15);

     painterXY->setPen(QPen(QBrush(Qt::blue), 1));
     //os X pozioma
     painterXY->drawLine(0,pixmapXY->height()/2,pixmapXY->width(),pixmapXY->height()/2);
     painterXY->drawLine(0,pixmapXY->height()/2,10,pixmapXY->height()/2-7);
     painterXY->drawLine(0,pixmapXY->height()/2,10,pixmapXY->height()/2+7);
     //litera X
     painterXY->drawLine(5,pixmapXY->height()/2+8, 15,pixmapXY->height()/2+18);
     painterXY->drawLine(5,pixmapXY->height()/2+18, 15,pixmapXY->height()/2+8);
     //os Y pionowa
     painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2,pixmapXY->height());
     painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2-7,10);
     painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2+7,10);
     //litera Y
     painterXY->drawLine(pixmapXY->width()/2+8,5,pixmapXY->width()/2+13,10);
     painterXY->drawLine(pixmapXY->width()/2+18,5,pixmapXY->width()/2+13,10);
     painterXY->drawLine(pixmapXY->width()/2+13,10,pixmapXY->width()/2+13,15);

     painterYZ->setPen(QPen(QBrush(Qt::blue), 1));
     //os Y pozioma
     painterYZ->drawLine(0,pixmapYZ->height()/2,pixmapYZ->width(),pixmapYZ->height()/2);
     painterYZ->drawLine(0,pixmapYZ->height()/2,10,pixmapYZ->height()/2-7);
     painterYZ->drawLine(0,pixmapYZ->height()/2,10,pixmapYZ->height()/2+7);
     //litera Y
     painterYZ->drawLine(5,pixmapYZ->height()/2+8,10,pixmapYZ->height()/2+13);
     painterYZ->drawLine(15,pixmapYZ->height()/2+8,10,pixmapYZ->height()/2+13);
     painterYZ->drawLine(10,pixmapYZ->height()/2+13,10,pixmapYZ->height()/2+18);
     //os Z pionowa
     painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2,pixmapYZ->height());
     painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2-7,10);
     painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2+7,10);
     //litera Z
     painterYZ->drawLine(pixmapYZ->width()/2+8,5,pixmapYZ->width()/2+18,5);
     painterYZ->drawLine(pixmapYZ->width()/2+18,5,pixmapYZ->width()/2+8,15);
     painterYZ->drawLine(pixmapYZ->width()/2+8,15,pixmapYZ->width()/2+18,15);
     /*OPIS OSI*/
     ui->XYView->setPixmap(*pixmapXY);
     ui->YZView->setPixmap(*pixmapYZ);
     ui->XZView->setPixmap(*pixmapXZ);

     timer = new QTimer(this);


    /* set up validators */
    QIntValidator* l1Validator = new QIntValidator(0, 10000,ui->l1LineEdit);
    QIntValidator* l2Validator = new QIntValidator(0, 10000,ui->l2LineEdit);
    QIntValidator* l3Validator = new QIntValidator(0, 10000,ui->l3LineEdit);
    QIntValidator* l4Validator = new QIntValidator(0, 10000,ui->l4LineEdit);
    QIntValidator* l5Validator = new QIntValidator(0, 10000,ui->l5LineEdit);
    QIntValidator* l6Validator = new QIntValidator(0, 10000,ui->l6LineEdit);
    QIntValidator* dValidator = new QIntValidator(0, 10000,ui->dLineEdit);
    QIntValidator* eValidator = new QIntValidator(0, 10000,ui->eLineEdit);
    QIntValidator* psiValidator = new QIntValidator(0, 360,ui->psiLineEdit);
    QIntValidator* thetaValidator = new QIntValidator(0, 360,ui->thetaLineEdit);

    ui->l1LineEdit->setValidator(l1Validator);
    ui->l2LineEdit->setValidator(l2Validator);
    ui->l3LineEdit->setValidator(l3Validator);
    ui->l4LineEdit->setValidator(l4Validator);
    ui->l5LineEdit->setValidator(l5Validator);
    ui->l6LineEdit->setValidator(l6Validator);
    ui->dLineEdit->setValidator(dValidator);
    ui->eLineEdit->setValidator(eValidator);
    ui->psiLineEdit->setValidator(psiValidator);
    ui->thetaLineEdit->setValidator(thetaValidator);

    /* set up signals'n'slots */
    connect(ui->updateButton, SIGNAL(clicked()), this, SLOT(on_updateButton_clicked()));
    connect(ui->calcTrajectoryButton, SIGNAL(clicked()),this, SLOT(on_calcTrajectoryButton_clicked()));
    connect(&kinematics, SIGNAL(wrongTCP()), wrongTrajectoryMessage, SLOT(exec()));
    connect(trajectoryDialog, SIGNAL(trajectoryEditFinished(Trajectory*)), this, SLOT(on_trajectoryEditFinished(Trajectory*)));
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));
    /* update kinematics with default values */
    updateKinematics();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete painterXY;
    delete painterXZ;
    delete painterYZ;
    delete pixmapXY;
    delete pixmapYZ;
    delete pixmapXZ;
}

double MainWindow::radToDeg(double rad)
{
    return rad * (180/M_PI);
}

double MainWindow::degToRad(double deg)
{
    return deg*M_PI/180;
}

void MainWindow::setPlotData1(QList<machineCoordinates> *coords)
{
    fi1 = new QVector<double>();
    fi2 = new QVector<double>();
    fi3 = new QVector<double>();
    fi4 = new QVector<double>();
    fi5 = new QVector<double>();
    time = new QVector<double>();

    for(int i = 0; i<coords->size();i++)
    {
        fi1->append(MainWindow::radToDeg((coords->at(i)).fi1));
        fi2->append(MainWindow::radToDeg((coords->at(i)).fi2));
        fi3->append(MainWindow::radToDeg((coords->at(i)).fi3));
        fi4->append(MainWindow::radToDeg((coords->at(i)).fi4));
        fi5->append(MainWindow::radToDeg((coords->at(i)).fi5));
        time->append(i);
    }

    ui->cplot->xAxis->setRange(0,time->last());
    ui->cplot->yAxis->setRange(-1* 90, 90);
    ui->cplot->yAxis->setLabel("Współrzędna maszynowa [˚]");
    ui->cplot->xAxis->setLabel("Czas");
    ui->cplot->xAxis->setTickLabels(false);
    ui->cplot->clearGraphs();
    ui->cplot->addGraph();
    ui->cplot->graph(0)->setData(*time,*fi1);
    ui->cplot->graph(0)->setPen(QPen(Qt::blue));
    ui->cplot->graph(0)->setName("φ1");
    ui->cplot->addGraph();
    ui->cplot->graph(1)->setData(*time,*fi2);
    ui->cplot->graph(1)->setPen(QPen(Qt::green));
    ui->cplot->graph(1)->setName("φ2");
    ui->cplot->addGraph();
    ui->cplot->graph(2)->setData(*time,*fi3);
    ui->cplot->graph(2)->setPen(QPen(Qt::red));
    ui->cplot->graph(2)->setName("φ3");
    ui->cplot->addGraph();
    ui->cplot->graph(3)->setData(*time,*fi4);
    ui->cplot->graph(3)->setPen(QPen(Qt::magenta));
    ui->cplot->graph(3)->setName("φ4");
    ui->cplot->addGraph();
    ui->cplot->graph(4)->setData(*time,*fi5);
    ui->cplot->graph(4)->setPen(QPen(Qt::black));
    ui->cplot->graph(4)->setName("φ5");
    ui->cplot->legend->setIconSize(5,5);

    ui->cplot->axisRect()->setAutoMargins(QCP::msLeft | QCP::msTop | QCP::msBottom);
    ui->cplot->axisRect()->setMargins(QMargins(0,0,60,0));
    ui->cplot->axisRect()->insetLayout()->setInsetPlacement(0, QCPLayoutInset::ipFree);
    ui->cplot->axisRect()->insetLayout()->setInsetRect(0, QRectF(1.05,0,0,0));
//  ui->cplot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
    ui->cplot->legend->setVisible(true);	
    ui->cplot->replot();

    delete fi1,fi2,fi3,fi4,fi5,time;
}

void MainWindow::setBasicQPlot()
{
    ui->cplot->xAxis->setRange(0,1);
    ui->cplot->yAxis->setRange(-1* 90, 90);
    ui->cplot->yAxis->setLabel("Współrzędna maszynowa [˚]");
    ui->cplot->xAxis->setLabel("Czas");
    ui->cplot->axisRect()->setAutoMargins(QCP::msLeft | QCP::msTop | QCP::msBottom);
    ui->cplot->axisRect()->setMargins(QMargins(0,0,60,0));
    ui->cplot->axisRect()->insetLayout()->setInsetPlacement(0, QCPLayoutInset::ipFree);
    ui->cplot->axisRect()->insetLayout()->setInsetRect(0, QRectF(1.05,0,0,0));
    ui->cplot->xAxis->setTickLabels(false);
}

void MainWindow::updateKinematics()
{
    aV.psi = degToRad((ui->psiLineEdit->text()).toDouble());
    aV.theta = degToRad((ui->thetaLineEdit->text()).toDouble());

    rPRegional.l1 = (ui->l1LineEdit->text()).toDouble();
    rPRegional.l2 = (ui->l2LineEdit->text()).toDouble();
    rPRegional.l3 = (ui->l3LineEdit->text()).toDouble();
    rPRegional.d = (ui->dLineEdit->text()).toDouble();
    rPRegional.e = (ui->eLineEdit->text()).toDouble();

    rPLocal.l4 = (ui->l4LineEdit->text()).toDouble();
    rPLocal.l5 = (ui->l5LineEdit->text()).toDouble();
    rPLocal.l6 = (ui->l6LineEdit->text()).toDouble();

    deltas.d1 = ui->d1SpinBox->value();
    deltas.d2 = ui->d2SpinBox->value();
    deltas.d5 = ui->d5SpinBox->value();

    kinematics.setApproachVector(aV);
    kinematics.setDeltas(deltas);
    kinematics.setRobotParamsLocal(rPLocal);
    kinematics.setRobotParamsRegional(rPRegional);
}

point3D MainWindow::getMaxCoords()
{
    point3D max;
    max.x=0;
    max.y=0;
    max.z=0;
    for(int i=0; i<resultCoordinates->size(); i++)
    {
        if(resultCoordinates->at(i).cartesian.p5.x>max.x)
            max.x=resultCoordinates->at(i).cartesian.p5.x;
        if(resultCoordinates->at(i).cartesian.p5.y>max.y)
            max.y=resultCoordinates->at(i).cartesian.p5.y;
        if(resultCoordinates->at(i).cartesian.p5.z>max.z)
            max.z=resultCoordinates->at(i).cartesian.p5.z;
    }
    return max;
}

void MainWindow::paintXY(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterXY->setPen(QPen(QBrush(Qt::black), 3));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.x,resultCoordinates->at(i).cartesian.p1.y);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.x,resultCoordinates->at(i).cartesian.p1pr.y);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.x,resultCoordinates->at(i).cartesian.p2.y);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.x,resultCoordinates->at(i).cartesian.p2pr.y);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.x,resultCoordinates->at(i).cartesian.p3.y);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.x,resultCoordinates->at(i).cartesian.p4.y);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.x,resultCoordinates->at(i).cartesian.p5.y);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapXY->width()/2, pixmapXY->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXY->width() - p1.x());
    p1.setY(pixmapXY->height() - p1.y());
    p1pr.setX(pixmapXY->width() - p1pr.x());
    p1pr.setY(pixmapXY->height() - p1pr.y());
    p2.setX(pixmapXY->width() - p2.x());
    p2.setY(pixmapXY->height() - p2.y());
    p2pr.setX(pixmapXY->width() - p2pr.x());
    p2pr.setY(pixmapXY->height() - p2pr.y());
    p3.setX(pixmapXY->width() - p3.x());
    p3.setY(pixmapXY->height() - p3.y());
    p4.setX(pixmapXY->width() - p4.x());
    p4.setY(pixmapXY->height() - p4.y());
    p5.setX(pixmapXY->width() - p5.x());
    p5.setY(pixmapXY->height() - p5.y());

    painterXY->eraseRect(0,0,pixmapXY->width(), pixmapXY->height());
    pixmapXY->fill(QColor("lightGray"));
    painterXY->drawLine(p0,p1);
    painterXY->drawLine(p1, p1pr);
    painterXY->drawLine(p1pr, p2pr);
    painterXY->drawLine(p2pr, p2);
    painterXY->drawLine(p2, p3);
    painterXY->drawLine(p3, p4);
    painterXY->drawLine(p4, p5);

    painterXY->setPen(QPen(QBrush(Qt::red), 4));
    painterXY->drawPoint(p5);
    painterXY->setPen(QPen(QBrush(Qt::yellow), 4));
    painterXY->drawPoint(p3);
    painterXY->setPen(QPen(QBrush(Qt::blue), 1));
    //os X pozioma
    painterXY->drawLine(0,pixmapXY->height()/2,pixmapXY->width(),pixmapXY->height()/2);
    painterXY->drawLine(0,pixmapXY->height()/2,10,pixmapXY->height()/2-7);
    painterXY->drawLine(0,pixmapXY->height()/2,10,pixmapXY->height()/2+7);
    //litera X
    painterXY->drawLine(5,pixmapXY->height()/2+8, 15,pixmapXY->height()/2+18);
    painterXY->drawLine(5,pixmapXY->height()/2+18, 15,pixmapXY->height()/2+8);
    //os Y pionowa
    painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2,pixmapXY->height());
    painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2-7,10);
    painterXY->drawLine(pixmapXY->width()/2,0,pixmapXY->width()/2+7,10);
    //litera Y
    painterXY->drawLine(pixmapXY->width()/2+8,5,pixmapXY->width()/2+13,10);
    painterXY->drawLine(pixmapXY->width()/2+18,5,pixmapXY->width()/2+13,10);
    painterXY->drawLine(pixmapXY->width()/2+13,10,pixmapXY->width()/2+13,15);
    painterXY->setPen(QPen(QBrush(Qt::white), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).x,trajectory->getTrajectory()->at(i).y);
        QPointF b(trajectory->getTrajectory()->at(i+1).x,trajectory->getTrajectory()->at(i+1).y);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapXY->width() - a.x());
        a.setY(pixmapXY->height() - a.y());
        b.setX(pixmapXY->width() - b.x());
        b.setY(pixmapXY->height() - b.y());
        painterXY->drawLine(a,b);
    }
    ui->XYView->setPixmap(*pixmapXY);


}

void MainWindow::paintYZ(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterYZ->setPen(QPen(QBrush(Qt::black), 3));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.y,resultCoordinates->at(i).cartesian.p1.z);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.y,resultCoordinates->at(i).cartesian.p1pr.z);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.y,resultCoordinates->at(i).cartesian.p2.z);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.y,resultCoordinates->at(i).cartesian.p2pr.z);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.y,resultCoordinates->at(i).cartesian.p3.z);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.y,resultCoordinates->at(i).cartesian.p4.z);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.y,resultCoordinates->at(i).cartesian.p5.z);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapYZ->width()/2, pixmapYZ->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXY->width() - p1.x());
    p1.setY(pixmapXY->height() - p1.y());
    p1pr.setX(pixmapXY->width() - p1pr.x());
    p1pr.setY(pixmapXY->height() - p1pr.y());
    p2.setX(pixmapXY->width() - p2.x());
    p2.setY(pixmapXY->height() - p2.y());
    p2pr.setX(pixmapXY->width() - p2pr.x());
    p2pr.setY(pixmapXY->height() - p2pr.y());
    p3.setX(pixmapXY->width() - p3.x());
    p3.setY(pixmapXY->height() - p3.y());
    p4.setX(pixmapXY->width() - p4.x());
    p4.setY(pixmapXY->height() - p4.y());
    p5.setX(pixmapXY->width() - p5.x());
    p5.setY(pixmapXY->height() - p5.y());

    painterYZ->eraseRect(0,0,pixmapYZ->width(), pixmapYZ->height());
    pixmapYZ->fill(QColor("lightGrey"));
    painterYZ->drawLine(p0,p1);
    painterYZ->drawLine(p1, p1pr);
    painterYZ->drawLine(p1pr, p2pr);
    painterYZ->drawLine(p2pr, p2);
    painterYZ->drawLine(p2, p3);
    painterYZ->drawLine(p3, p4);
    painterYZ->drawLine(p4, p5);
    painterYZ->setPen(QPen(QBrush(Qt::red), 4));
    painterYZ->drawPoint(p5);
    painterYZ->setPen(QPen(QBrush(Qt::yellow), 4));
    painterYZ->drawPoint(p3);
    painterYZ->setPen(QPen(QBrush(Qt::blue), 1));
    //os Y pozioma
    painterYZ->drawLine(0,pixmapYZ->height()/2,pixmapYZ->width(),pixmapYZ->height()/2);
    painterYZ->drawLine(0,pixmapYZ->height()/2,10,pixmapYZ->height()/2-7);
    painterYZ->drawLine(0,pixmapYZ->height()/2,10,pixmapYZ->height()/2+7);
    //litera Y
    painterYZ->drawLine(5,pixmapYZ->height()/2+8,10,pixmapYZ->height()/2+13);
    painterYZ->drawLine(15,pixmapYZ->height()/2+8,10,pixmapYZ->height()/2+13);
    painterYZ->drawLine(10,pixmapYZ->height()/2+13,10,pixmapYZ->height()/2+18);
    //os Z pionowa
    painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2,pixmapYZ->height());
    painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2-7,10);
    painterYZ->drawLine(pixmapYZ->width()/2,0,pixmapYZ->width()/2+7,10);
    //litera Z
    painterYZ->drawLine(pixmapYZ->width()/2+8,5,pixmapYZ->width()/2+18,5);
    painterYZ->drawLine(pixmapYZ->width()/2+18,5,pixmapYZ->width()/2+8,15);
    painterYZ->drawLine(pixmapYZ->width()/2+8,15,pixmapYZ->width()/2+18,15);

    painterYZ->setPen(QPen(QBrush(Qt::white), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).y,trajectory->getTrajectory()->at(i).z);
        QPointF b(trajectory->getTrajectory()->at(i+1).y,trajectory->getTrajectory()->at(i+1).z);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapYZ->width() - a.x());
        a.setY(pixmapYZ->height() - a.y());
        b.setX(pixmapYZ->width() - b.x());
        b.setY(pixmapYZ->height() - b.y());
        painterYZ->drawLine(a,b);
    }
    ui->YZView->setPixmap(*pixmapYZ);
}

void MainWindow::paintXZ(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterXZ->setPen(QPen(QBrush(Qt::black), 3));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.x,resultCoordinates->at(i).cartesian.p1.z);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.x,resultCoordinates->at(i).cartesian.p1pr.z);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.x,resultCoordinates->at(i).cartesian.p2.z);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.x,resultCoordinates->at(i).cartesian.p2pr.z);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.x,resultCoordinates->at(i).cartesian.p3.z);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.x,resultCoordinates->at(i).cartesian.p4.z);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.x,resultCoordinates->at(i).cartesian.p5.z);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapYZ->width()/2, pixmapYZ->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXZ->width() - p1.x());
    p1.setY(pixmapXZ->height() - p1.y());
    p1pr.setX(pixmapXZ->width() - p1pr.x());
    p1pr.setY(pixmapXZ->height() - p1pr.y());
    p2.setX(pixmapXZ->width() - p2.x());
    p2.setY(pixmapXZ->height() - p2.y());
    p2pr.setX(pixmapXZ->width() - p2pr.x());
    p2pr.setY(pixmapXZ->height() - p2pr.y());
    p3.setX(pixmapXZ->width() - p3.x());
    p3.setY(pixmapXZ->height() - p3.y());
    p4.setX(pixmapXZ->width() - p4.x());
    p4.setY(pixmapXZ->height() - p4.y());
    p5.setX(pixmapXZ->width() - p5.x());
    p5.setY(pixmapXZ->height() - p5.y());

    painterXZ->eraseRect(0,0,pixmapYZ->width(), pixmapYZ->height());
    pixmapXZ->fill(QColor("lightGray"));
    painterXZ->drawLine(p0,p1);
    painterXZ->drawLine(p1, p1pr);
    painterXZ->drawLine(p1pr, p2pr);
    painterXZ->drawLine(p2pr, p2);
    painterXZ->drawLine(p2, p3);
    painterXZ->drawLine(p3, p4);
    painterXZ->drawLine(p4, p5);
    painterXZ->setPen(QPen(QBrush(Qt::red), 4));
    painterXZ->drawPoint(p5);
    painterXZ->setPen(QPen(QBrush(Qt::yellow), 4));
    painterXZ->drawPoint(p3);
    painterXZ->setPen(QPen(QBrush(Qt::blue), 1));
    //os X pozioma
    painterXZ->drawLine(0,pixmapXZ->height()/2,pixmapXZ->width(),pixmapXZ->height()/2);
    painterXZ->drawLine(0,pixmapXZ->height()/2,10,pixmapXZ->height()/2-7);
    painterXZ->drawLine(0,pixmapXZ->height()/2,10,pixmapXZ->height()/2+7);
    //litera X
    painterXZ->drawLine(5,pixmapXZ->height()/2+8, 15,pixmapXZ->height()/2+18);
    painterXZ->drawLine(5,pixmapXZ->height()/2+18, 15,pixmapXZ->height()/2+8);
    //os Z pionowa
    painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2,pixmapXZ->height());
    painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2-7,10);
    painterXZ->drawLine(pixmapXZ->width()/2,0,pixmapXZ->width()/2+7,10);
    //litera Z
    painterXZ->drawLine(pixmapXZ->width()/2+8,5,pixmapXZ->width()/2+18,5);
    painterXZ->drawLine(pixmapXZ->width()/2+18,5,pixmapXZ->width()/2+8,15);
    painterXZ->drawLine(pixmapXZ->width()/2+8,15,pixmapXZ->width()/2+18,15);
    painterXZ->setPen(QPen(QBrush(Qt::white), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).x,trajectory->getTrajectory()->at(i).z);
        QPointF b(trajectory->getTrajectory()->at(i+1).x,trajectory->getTrajectory()->at(i+1).z);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapXZ->width() - a.x());
        a.setY(pixmapXZ->height() - a.y());
        b.setX(pixmapXZ->width() - b.x());
        b.setY(pixmapXZ->height() - b.y());
        painterXZ->drawLine(a,b);
    }
    ui->XZView->setPixmap(*pixmapXZ);
}

void MainWindow::on_updateButton_clicked()
{
    updateKinematics();
}

void MainWindow::on_calcTrajectoryButton_clicked()
{
    ui->animationButton->setEnabled(false);
    ui->horizontalSlider->setEnabled(false);
    trajectoryDialog->reset();
    trajectoryDialog->show();

}

void MainWindow::on_trajectoryEditFinished(Trajectory *trajectory)
{
    this->trajectory = trajectory;
    kinematics.setTrajectory(trajectory);
    resultCoordinates = kinematics.getMachineCoordinates();

    if(resultCoordinates != NULL){
        ui->animationButton->setEnabled(true);
        ui->horizontalSlider->setRange(0,resultCoordinates->size()-1);
        ui->horizontalSlider->setEnabled(true);
        point3D maxP = getMaxCoords();
        maxP.x+=150;
        maxP.y+=150;
        maxP.z+=150;
        factor = ui->XYView->size().width() / std::max(maxP.x, std::max(maxP.z,maxP.y));
        factor /= 2;

    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    paintXY(value);
    paintYZ(value);
    paintXZ(value);
}

void MainWindow::on_animationButton_clicked()
{
    int interval = (11-ui->animSpeedBox->value())*100;
    timer->start(interval);
    ui->horizontalSlider->setValue(0);
    this->setPlotData1(resultCoordinates);                      //plotterdialog
}

void MainWindow::on_timer_timeout()
{
    int val = ui->horizontalSlider->value();
    if(val<resultCoordinates->size()-1)
        ui->horizontalSlider->setValue(++val);
    else
    {
        ui->horizontalSlider->setValue(0);
        timer->stop();
    }
}

void MainWindow::on_infoButton_clicked()
{
    infoDialog->show();
}
