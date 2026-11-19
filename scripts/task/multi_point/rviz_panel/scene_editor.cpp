#include "scene_editor.hpp"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QFrame>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QSignalBlocker>
#include <QStyle>
#include <cmath>
#include <algorithm>

namespace piper_multi_point_rviz {
SceneEditor::SceneEditor(QWidget *parent) : QWidget(parent) {
  auto *root=new QVBoxLayout(this);root->setContentsMargins(0,0,0,0);root->setSpacing(8);
  auto *scroll=new QScrollArea(this);scroll->setWidgetResizable(true);scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);root->addWidget(scroll,1);
  auto *content=new QWidget;scroll->setWidget(content);auto *layout=new QVBoxLayout(content);layout->setContentsMargins(8,10,8,10);layout->setSpacing(12);layout->setAlignment(Qt::AlignTop);
  auto *fileTitle=new QLabel("场景文件",content);fileTitle->setProperty("role","sectionTitle");layout->addWidget(fileTitle);
  auto *files=new QHBoxLayout;load_=new QPushButton("加载场景");save_=new QPushButton("另存为…");
  files->addWidget(load_);files->addWidget(save_);layout->addLayout(files);
  load_->setToolTip("加载场景文件为草稿；应用场景后才参与碰撞检测");save_->setToolTip("将已应用的场景保存为新的 YAML 文件");
  status_=new QLabel;status_->setObjectName("sceneStatus");status_->setProperty("role","status");status_->setWordWrap(true);
  status_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);layout->addWidget(status_);
  auto *objectTitle=new QLabel("物体草稿",content);objectTitle->setProperty("role","sectionTitle");layout->addWidget(objectTitle);
  editing_=new QCheckBox("拖拽编辑");editing_->setToolTip("仅显示选中物体的拖拽手柄；应用后才参与碰撞检测");layout->addWidget(editing_);
  objects_=new QComboBox;objects_->setObjectName("sceneObjects");layout->addWidget(objects_);
  auto *actions=new QHBoxLayout;add_=new QPushButton("新增");remove_=new QPushButton("删除");
  actions->addWidget(add_);actions->addWidget(remove_);layout->addLayout(actions);
  form_=new QWidget;auto *form=new QFormLayout(form_);form->setContentsMargins(0,0,0,0);form->setSpacing(8);
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);form->setFormAlignment(Qt::AlignTop);
  id_=new QLineEdit;id_->setObjectName("objectId");id_->setPlaceholderText("字母、数字、_ 或 -");id_->setMinimumWidth(0);form->addRow("名称",id_);
  shape_=new QComboBox;shape_->addItem("箱体","box");shape_->addItem("球体","sphere");shape_->addItem("圆柱","cylinder");form->addRow("形状",shape_);
  form_->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);layout->addWidget(form_);
  auto row=[&](const QString &label,const QStringList &axes,QDoubleSpinBox **fields,double lower,double upper,int decimals,bool dimensions=false) {
    auto *title=new QLabel(label,content);title->setProperty("role","sectionTitle");layout->addWidget(title);
    auto *widget=new QWidget;auto *line=new QGridLayout(widget);line->setContentsMargins(0,0,0,0);line->setHorizontalSpacing(6);line->setVerticalSpacing(3);
    widget->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
    for(int i=0;i<3;++i){
      auto *axis=new QLabel(axes[i],widget);axis->setProperty("role","muted");if(dimensions)dimension_labels_[i]=axis;
      fields[i]=new QDoubleSpinBox;fields[i]->setRange(lower,upper);fields[i]->setDecimals(decimals);fields[i]->setSingleStep(decimals==1?5.:.01);
      fields[i]->setMinimumWidth(0);fields[i]->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);fields[i]->setAccessibleName(label+" "+axes[i]);
      line->addWidget(axis,0,i);line->addWidget(fields[i],1,i);line->setColumnStretch(i,1);
    }
    layout->addWidget(widget);
  };
  row("位置 · m",{"X","Y","Z"},position_,-100.,100.,3);
  row("姿态 · °",{"Roll","Pitch","Yaw"},rotation_,-180.,180.,1);
  row("尺寸 · m",{"长","宽","高"},dimensions_,.001,100.,3,true);
  for(int i=0;i<3;++i){position_[i]->setObjectName("position"+QString::number(i));rotation_[i]->setObjectName("rotation"+QString::number(i));dimensions_[i]->setObjectName("dimension"+QString::number(i));}
  update_=new QPushButton("写入草稿");apply_=new QPushButton("应用场景");apply_->setProperty("role","primary");
  auto *commit=new QHBoxLayout;commit->setSpacing(8);commit->addWidget(update_,1);commit->addWidget(apply_,1);layout->addLayout(commit);
  update_->setToolTip("将表单数值写入草稿；再点击应用场景以更新碰撞检测");
  apply_->setToolTip("应用完整草稿并关闭拖拽手柄；旧预览作废，需要重新规划");
  update_->setObjectName("writeDraft");apply_->setObjectName("applyScene");
  for(auto *button:{load_,save_,add_,remove_,update_,apply_}){button->setMinimumHeight(52);button->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);}
  layout->addStretch();
  connect(load_,&QPushButton::clicked,this,[this]{auto path=QFileDialog::getOpenFileName(this,"加载场景",directory_,"YAML (*.yaml *.yml)");if(!path.isEmpty()) emit command("scene_load",path);});
  connect(save_,&QPushButton::clicked,this,[this]{auto path=QFileDialog::getSaveFileName(this,"场景另存为（使用新文件名）",QDir(directory_).filePath("scene_custom.yaml"),"YAML (*.yaml *.yml)");if(!path.isEmpty()) emit command("scene_save",path);});
  connect(editing_,&QCheckBox::toggled,this,[this](bool value){emit command("scene_edit",value?"true":"false");});
  connect(objects_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{fillObject();emit command("scene_select",objects_->currentText());});
  connect(shape_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{updateDimensions();});
  connect(add_,&QPushButton::clicked,this,[this]{
    objects_->setCurrentIndex(-1);id_->setReadOnly(false);int number=1;QString name;
    do{name="object_"+QString::number(number++);}while(objects_->findText(name)>=0);
    id_->setText(name);shape_->setCurrentIndex(0);
    for(int i=0;i<3;++i){position_[i]->setValue(i==0?.3:i==2?.4:0.);rotation_[i]->setValue(0.);dimensions_[i]->setValue(.1);}
  });
  connect(remove_,&QPushButton::clicked,this,[this]{if(objects_->currentIndex()>=0)emit command("scene_delete",objects_->currentText());});
  connect(update_,&QPushButton::clicked,this,&SceneEditor::submitObject);
  connect(apply_,&QPushButton::clicked,this,[this]{emit command("scene_apply","");});
  for(auto *field:dimensions_)field->setValue(.1);
  updateDimensions();setEnabled(false);
}
void SceneEditor::updateDimensions(){
  const auto shape=shape_->currentData().toString();int count=shape=="box"?3:shape=="sphere"?1:2;
  const QStringList names=shape=="box"?QStringList{"长","宽","高"}:shape=="sphere"?QStringList{"半径"}:QStringList{"高度","半径"};
  auto *grid=qobject_cast<QGridLayout*>(dimensions_[0]->parentWidget()->layout());
  for(int i=0;i<3;++i){dimensions_[i]->setVisible(i<count);dimension_labels_[i]->setVisible(i<count);grid->setColumnStretch(i,i<count?1:0);if(i<count){dimension_labels_[i]->setText(names[i]);dimensions_[i]->setToolTip(names[i]+"（米）");dimensions_[i]->setAccessibleName(names[i]+"，米");}}
}
void SceneEditor::fillObject(){
  const int index=objects_->currentIndex();if(index<0 || index>=data_.size())return;
  const auto obj=data_[index].toObject();id_->setText(obj["id"].toString());id_->setReadOnly(true);
  shape_->setCurrentIndex(shape_->findData(obj["shape"].toString()));frame_=obj["frame_id"].toString("base_link");
  const auto p=obj["position_m"].toArray(),d=obj["dimensions_m"].toArray(),q=obj["orientation_xyzw"].toArray();
  for(int i=0;i<3;++i){position_[i]->setValue(p[i].toDouble());dimensions_[i]->setValue(i<d.size()?d[i].toDouble():.1);}
  if(q.size()==4){double x=q[0].toDouble(),y=q[1].toDouble(),z=q[2].toDouble(),w=q[3].toDouble();
    rotation_[0]->setValue(std::atan2(2*(w*x+y*z),1-2*(x*x+y*y))*180/M_PI);
    rotation_[1]->setValue(std::asin(std::clamp(2*(w*y-z*x),-1.,1.))*180/M_PI);
    rotation_[2]->setValue(std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))*180/M_PI);}
}
void SceneEditor::submitObject(){
  QJsonArray pos,dims;for(auto *field:position_)pos.append(field->value());
  const auto shape=shape_->currentData().toString();int count=shape=="box"?3:shape=="sphere"?1:2;
  for(int i=0;i<count;++i)dims.append(dimensions_[i]->value());
  double r=rotation_[0]->value()*M_PI/360,p=rotation_[1]->value()*M_PI/360,y=rotation_[2]->value()*M_PI/360;
  double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);
  QJsonArray q{sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};
  QJsonObject obj{{"id",id_->text()},{"shape",shape},{"frame_id",frame_},{"position_m",pos},{"dimensions_m",dims},{"orientation_xyzw",q}};
  emit command("scene_upsert",QString::fromUtf8(QJsonDocument(obj).toJson(QJsonDocument::Compact)));
}
void SceneEditor::updateState(const QJsonObject &state,bool idle){
  setEnabled(idle && state["scene_editor"].toBool());
  const auto config=state["config_path"].toString();file_=state["scene_file"].toString();
  if(!file_.isEmpty())directory_=QFileInfo(file_).absolutePath();else if(!config.isEmpty())directory_=QFileInfo(config).absolutePath();
  {QSignalBlocker blocker(editing_);editing_->setChecked(state["scene_editing"].toBool());}
  const int revision=state["scene_revision"].toInt();
  const auto selected=state["scene_selected"].toString();
  if(revision!=revision_ || selected!=selection_){revision_=revision;selection_=selected;data_=state["scene_objects"].toArray();
    {QSignalBlocker blocker(objects_);objects_->clear();for(auto value:data_)objects_->addItem(value.toObject()["id"].toString());objects_->setCurrentIndex(objects_->findText(selected));}fillObject();}
  const bool dirty=state["scene_dirty"].toBool();save_->setEnabled(idle && !dirty);remove_->setEnabled(idle && objects_->currentIndex()>=0);
  status_->setText(dirty?"草稿待应用":state["scene_editing"].toBool()?"拖拽编辑中":"场景已应用");
  status_->setToolTip(file_);
  if(state["state"].toString()=="ERROR"){
    status_->setText("操作未完成 · 悬停查看");
    status_->setToolTip(state["message"].toString()+"\n"+file_);
  }
  const QString tone=state["state"].toString()=="ERROR"?"danger":dirty || state["scene_editing"].toBool()?"warning":"active";
  if(status_->property("tone").toString()!=tone) {
    status_->setProperty("tone",tone);status_->style()->unpolish(status_);status_->style()->polish(status_);
  }
}
}
