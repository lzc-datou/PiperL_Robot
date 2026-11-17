#include "scripts/task/multi_point/rviz_panel/scene_editor.hpp"
#include <QApplication>
#include <QJsonDocument>
#include <stdexcept>
#include <cmath>
using piper_multi_point_rviz::SceneEditor;
void check(bool ok,const char *msg){if(!ok)throw std::runtime_error(msg);}
int main(int argc,char **argv){
  QApplication app(argc,argv);SceneEditor editor;editor.resize(400,600);
  QJsonObject object{{"id","box"},{"shape","box"},{"frame_id","base_link"},
    {"position_m",QJsonArray{.3,.1,.4}},{"dimensions_m",QJsonArray{.1,.2,.3}},
    {"orientation_xyzw",QJsonArray{0.,0.,0.,1.}}};
  QJsonObject state{{"scene_editor",true},{"scene_revision",1},{"scene_editing",false},
    {"scene_dirty",false},{"scene_file","/tmp/config/scene.yaml"},{"scene_selected","box"},{"scene_objects",QJsonArray{object}}};
  editor.updateState(state,true);editor.show();app.processEvents();
  QString command,payload;QObject::connect(&editor,&SceneEditor::command,[&](const QString &c,const QString &p){command=c;payload=p;});
  auto sphere=object;sphere["id"]="sphere";sphere["position_m"]=QJsonArray{.6,.1,.4};
  state["scene_objects"]=QJsonArray{object,sphere};state["scene_revision"]=2;
  editor.updateState(state,true);
  auto *objects=editor.findChild<QComboBox*>("sceneObjects");
  objects->setCurrentIndex(1);
  check(command=="scene_select" && payload=="sphere","Dropdown did not select backend object");
  command.clear();state["scene_selected"]="sphere";editor.updateState(state,true);
  check(command.isEmpty(),"Status refresh must not send selection back");
  state["scene_selected"]="box";editor.updateState(state,true);
  check(objects->currentText()=="box" && editor.findChild<QDoubleSpinBox*>("position0")->value()==.3,"3D selection did not update form");
  editor.findChild<QDoubleSpinBox*>("position0")->setValue(.45);
  editor.findChild<QDoubleSpinBox*>("rotation2")->setValue(90.);
  editor.findChild<QPushButton*>("writeDraft")->click();
  check(command=="scene_upsert","Numeric editor did not submit a draft");
  const auto result=QJsonDocument::fromJson(payload.toUtf8()).object();
  check(result["position_m"].toArray()[0].toDouble()==.45,"Wrong position units");
  check(std::abs(result["orientation_xyzw"].toArray()[2].toDouble()-std::sqrt(.5))<1e-9,"Wrong rotation conversion");
  check(result["dimensions_m"].toArray()==QJsonArray{.1,.2,.3},"Dimension mapping changed");
  editor.findChild<QPushButton*>("applyScene")->click();check(command=="scene_apply","Apply did not emit explicit command");
  editor.updateState(state,false);check(!editor.isEnabled(),"Busy editor must be disabled");
  state["state"]="ERROR";state["message"]=QString(2000,'x')+"\n/tmp/long/log/path";
  editor.updateState(state,true);
  auto *status=editor.findChild<QLabel*>("sceneStatus");
  check(status && !status->text().contains('\n') && status->text().size()<30,"Error expands scene panel");
  check(status->toolTip().contains(state["message"].toString()),"Detailed error must remain available");
  state["state"]="IDLE";
  editor.updateState(state,true);app.processEvents();if(argc>1)check(editor.grab().save(argv[1]),"Screenshot failed");
  return 0;
}
