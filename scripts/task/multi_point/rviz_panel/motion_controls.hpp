#pragma once
#include <QWidget>
#include <QJsonObject>
#include <QMap>
#include <QPushButton>
#include <QGridLayout>

namespace piper_multi_point_rviz {
class MotionControls : public QWidget {
  Q_OBJECT
public:
  explicit MotionControls(QWidget *parent=nullptr);
  void updateState(const QJsonObject &state, bool online, bool idle,
                   bool selected_loaded, bool preview_visible);
signals:
  void command(const QString &name);
private:
  void arrange(bool comparison);
  QMap<QString,QPushButton*> buttons_;
  QGridLayout *grid_;
  bool comparison_=false;
};
}
