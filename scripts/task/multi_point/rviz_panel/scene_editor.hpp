#pragma once
#include <QWidget>
#include <QJsonObject>
#include <QJsonArray>
#include <QComboBox>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>

namespace piper_multi_point_rviz {
class SceneEditor : public QWidget {
  Q_OBJECT
public:
  explicit SceneEditor(QWidget *parent=nullptr);
  void updateState(const QJsonObject &state, bool idle);
signals:
  void command(const QString &name, const QString &payload);
private:
  void fillObject();
  void updateDimensions();
  void submitObject();
  QComboBox *objects_, *shape_;
  QLineEdit *id_;
  QDoubleSpinBox *position_[3], *rotation_[3], *dimensions_[3];
  QCheckBox *editing_;
  QLabel *status_;
  QLabel *dimension_labels_[3];
  QWidget *form_;
  QPushButton *load_, *save_, *add_, *remove_, *update_, *apply_;
  QJsonArray data_;
  int revision_=-1;
  QString selection_;
  QString frame_="base_link", directory_, file_;
};
}
