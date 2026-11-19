#pragma once
#include <QFile>
#include <QFont>
#include <QEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QStyle>
#include <QStyleOptionFocusRect>
#include <QTextLayout>
#include <QWidget>

namespace piper_ui {
// Keep the command area stable even when a diagnostic contains a long sentence.
// The full label and diagnostic remain available by mouse, keyboard and tooltip.
class StatusLabel : public QLabel {
public:
  explicit StatusLabel(const QString &text, QWidget *parent=nullptr):QLabel(text,parent) {
    setTextFormat(Qt::PlainText);setMinimumWidth(0);
    setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    setFocusPolicy(Qt::StrongFocus);setCursor(Qt::PointingHandCursor);
    setAccessibleDescription("点击或按 Enter 查看完整状态");
    setFixedHeight(sizeHint().height());
  }
  QSize sizeHint() const override {return QSize(0,fontMetrics().lineSpacing()*2+8);}
  QSize minimumSizeHint() const override {return sizeHint();}
protected:
  void changeEvent(QEvent *event) override {
    QLabel::changeEvent(event);
    if(event->type()==QEvent::FontChange || event->type()==QEvent::StyleChange)
      setFixedHeight(sizeHint().height());
  }
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);painter.setFont(font());
    painter.setPen(palette().color(isEnabled()?QPalette::Active:QPalette::Disabled,foregroundRole()));
    const auto area=contentsRect().adjusted(2,2,-2,-2);
    const auto value=text().simplified();
    QTextLayout textLayout(value,font());
    QTextOption option;option.setWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    textLayout.setTextOption(option);textLayout.beginLayout();
    auto first=textLayout.createLine();
    if(first.isValid())first.setLineWidth(area.width());
    textLayout.endLayout();
    const int count=first.isValid()?first.textLength():0;
    const int lineHeight=fontMetrics().lineSpacing();
    if(count>=value.size()) {
      painter.drawText(area,Qt::AlignLeft|Qt::AlignVCenter|Qt::TextSingleLine,value);
    } else {
      painter.drawText(QRect(area.x(),area.y(),area.width(),lineHeight),Qt::AlignLeft|Qt::AlignVCenter,value.left(count).trimmed());
      painter.drawText(QRect(area.x(),area.y()+lineHeight,area.width(),lineHeight),Qt::AlignLeft|Qt::AlignVCenter,
        fontMetrics().elidedText(value.mid(count).trimmed(),Qt::ElideRight,area.width()));
    }
    if(hasFocus()) {
      QStyleOptionFocusRect focus;focus.initFrom(this);focus.rect=rect();
      style()->drawPrimitive(QStyle::PE_FrameFocusRect,&focus,&painter,this);
    }
  }
  void mouseReleaseEvent(QMouseEvent *event) override {
    if(event->button()==Qt::LeftButton && rect().contains(event->pos())) {showDetails();event->accept();}
    else QLabel::mouseReleaseEvent(event);
  }
  void keyPressEvent(QKeyEvent *event) override {
    if(event->key()==Qt::Key_Return || event->key()==Qt::Key_Enter || event->key()==Qt::Key_Space) {
      showDetails();event->accept();
    } else QLabel::keyPressEvent(event);
  }
private:
  void showDetails() {
    auto *box=findChild<QMessageBox *>("piperStatusDetails");
    if(!box) {
      box=new QMessageBox(QMessageBox::Information,"任务状态",text(),QMessageBox::Ok,this);
      box->setObjectName("piperStatusDetails");box->setAttribute(Qt::WA_DeleteOnClose);
      box->setTextFormat(Qt::PlainText);box->setWindowModality(Qt::NonModal);
    }
    box->setText(text());
    box->setInformativeText(toolTip()!=text()?toolTip():QString());
    box->show();box->raise();box->activateWindow();
  }
};

class AdaptivePanelTheme : public QObject {
public:
  explicit AdaptivePanelTheme(QWidget *panel):QObject(panel),panel_(panel) {
    setObjectName("piperAdaptiveTheme");
    QFile file(":/piper/rviz/theme.qss");
    if(file.open(QIODevice::ReadOnly))theme_=QString::fromUtf8(file.readAll());
    panel_->installEventFilter(this);update();
  }
protected:
  bool eventFilter(QObject *object,QEvent *event) override {
    if(object==panel_ && (event->type()==QEvent::Resize || event->type()==QEvent::Show))update();
    return QObject::eventFilter(object,event);
  }
private:
  void update() {
    const int band=panel_->width()>=560 && panel_->height()>=1200?2:
      panel_->width()>=460 && panel_->height()>=850?1:0;
    if(band==band_)return;
    band_=band;
    const int body[]={22,26,28},button[]={24,28,30},title[]={28,34,36};
    const int hint[]={20,22,24},control[]={52,64,72},field[]={40,48,52};
    QFont font=panel_->font();font.setPixelSize(body[band]);font.setWeight(QFont::Normal);
    panel_->setFont(font);
    auto sheet=theme_;
    const auto replace=[&](const QString &key,int value){sheet.replace("@"+key+"@",QString::number(value));};
    replace("body",body[band]);replace("button",button[band]);replace("title",title[band]);
    replace("hint",hint[band]);replace("control",control[band]);replace("stop",control[band]+8);replace("field",field[band]);
    panel_->setProperty("piperUiBodyPx",body[band]);panel_->setProperty("piperUiScale",band);
    panel_->setStyleSheet(sheet);
  }
  QWidget *panel_;
  QString theme_;
  int band_=-1;
};

// Keep the host RViz QStyle, palette and font family. Scope typography to this
// panel so the adjacent official MotionPlanning widgets retain their defaults.
inline void apply(QWidget *widget) {
  widget->setAutoFillBackground(true);
  if(!widget->findChild<QObject *>("piperAdaptiveTheme"))new AdaptivePanelTheme(widget);
}
}  // namespace piper_ui
