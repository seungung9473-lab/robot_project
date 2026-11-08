#include <stdexcept>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QApplication>
#include <QThread>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPointer>
#include <QListWidget>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QUuid>
#include <QElapsedTimer>
#include <QScrollArea>
#include <map>
#include <vector>
namespace soarm4_moveit_config {
class HomePanel:public rviz_common::Panel {
public:
 explicit HomePanel(QWidget* parent=nullptr):rviz_common::Panel(parent) {
  auto* root=new QVBoxLayout(this);auto* row=new QHBoxLayout();
  auto button=[&](const char* text,auto callback){auto* b=new QPushButton(QString::fromUtf8(text));connect(b,&QPushButton::clicked,this,callback);return b;};
  home_=button("HOME",[this](){request("home");});stop_=button("정지 (STOP)",[this](){request("stop");});torque_=button("토크 해제 / 고정",[this](){request("torque");});
  torque_->setToolTip(QString::fromUtf8("팔을 받친 상태에서 누르세요. 같은 버튼으로 토크 OFF/ON 전환"));torque_->setObjectName("soarm4_release_torque_button");
  row->addWidget(home_);row->addWidget(stop_);row->addWidget(torque_);root->addLayout(row);
  status_=new QLabel(QString::fromUtf8("관리 서비스 연결 대기"));status_->setWordWrap(true);root->addWidget(status_);
  auto* pump_row=new QHBoxLayout();pump_on_=button("흡착 ON (수동)",[this](){request("vacuum_on");});pump_release_=button("놓기 / 해제 (수동)",[this](){request("vacuum_release");});pump_on_->setObjectName("soarm4_manual_grip");pump_release_->setObjectName("soarm4_manual_release");pump_row->addWidget(pump_on_);pump_row->addWidget(pump_release_);root->addLayout(pump_row);pressure_=new QLabel(QString::fromUtf8("압력 diff: 수신 대기"));root->addWidget(pressure_);
  auto* scroll=new QScrollArea();scroll->setWidgetResizable(true);scroll->setMinimumHeight(210);auto* body=new QWidget();auto* layout=new QVBoxLayout(body);
  layout->addWidget(new QLabel(QString::fromUtf8("이동 자세와 흡착·놓기 단계를 순서대로 저장")));
  route_=new QListWidget();route_->setMinimumHeight(100);route_->setMaximumHeight(140);route_->setObjectName("soarm4_route_list");layout->addWidget(route_);
  auto* r1=new QHBoxLayout();save_=button("선택 자세 저장",[this](){saveSelected();});add_=button("자세 추가",[this](){insertAfter();});r1->addWidget(save_);r1->addWidget(add_);layout->addLayout(r1);
  auto* action_row=new QHBoxLayout();insert_grip_=button("흡착 단계 추가",[this](){insertAfter("insert_grip");});insert_release_=button("놓기 단계 추가",[this](){insertAfter("insert_release");});action_row->addWidget(insert_grip_);action_row->addWidget(insert_release_);layout->addLayout(action_row);
  auto* r2=new QHBoxLayout();up_=button("위로 ↑",[this](){moveSelected(-1);});down_=button("아래로 ↓",[this](){moveSelected(1);});remove_=button("선택 단계 삭제",[this](){editSelected("delete");});clear_=button("전체 초기화",[this](){sendEdit(QJsonObject{{"action","clear"}});});r2->addWidget(up_);r2->addWidget(down_);r2->addWidget(remove_);r2->addWidget(clear_);layout->addLayout(r2);
  auto* r3=new QHBoxLayout();list_=button("저장 확인",[this](){request("list");});check_=button("전체 경로 확인",[this](){request("check");});run_=button("1회 실행",[this](){request("run");});r3->addWidget(list_);r3->addWidget(check_);r3->addWidget(run_);
  scroll->setWidget(body);root->addWidget(scroll);root->addLayout(r3);controls_={home_,torque_,pump_on_,pump_release_,insert_grip_,insert_release_,save_,add_,up_,down_,remove_,clear_,list_,check_,run_};
  auto* timer=new QTimer(this);connect(timer,&QTimer::timeout,this,[this](){
   if(!pending_.isEmpty()&&elapsed_.elapsed()>10000){pending_.clear();select_index_=-1;busy_=false;status_->setText(QString::fromUtf8("편집 응답 시간 초과. 저장 확인으로 결과를 확인하세요."));}
   pressure_->setText(sensorless_?QString::fromUtf8("센서 미사용 · 시간 기준 실행 · 흡착 확인 없음"):!have_pressure_||pressure_elapsed_.elapsed()>1000?QString::fromUtf8("압력 diff: 수신 없음/지연"):QString::fromUtf8("압력 diff: %1 · %2").arg(diff_).arg(holding_?QString::fromUtf8("흡착 확인"):QString::fromUtf8("흡착 미확인")));
   for(auto* b:controls_)b->setEnabled(node_&&!busy_);
   stop_->setEnabled(node_&&clients_["stop"]&&clients_["stop"]->service_is_ready());
   if(!busy_){auto s=selected();bool waypoint=s.value("kind")=="waypoint";bool editable=waypoint||s.value("kind")=="action";save_->setEnabled(node_&&(waypoint||s.value("kind")=="anchor"));up_->setEnabled(node_&&editable);down_->setEnabled(node_&&editable);remove_->setEnabled(node_&&editable);add_->setEnabled(node_&&!s.isEmpty());}
  });timer->start(250);
 }
 void onInitialize() override {
  auto abstraction=getDisplayContext()->getRosNodeAbstraction().lock();if(!abstraction)return;node_=abstraction->get_raw_node();
  for(auto pair:{std::pair<const char*,const char*>("home","/soarm4/go_home"),{"stop","/soarm4/stop_home"},{"torque","/soarm4/toggle_torque"},{"list","/soarm4/pick_place/list"},{"check","/soarm4/pick_place/check"},{"run","/soarm4/pick_place/run"},{"vacuum_on","/soarm4/vacuum/grip"},{"vacuum_release","/soarm4/vacuum/release"}})clients_[pair.first]=node_->create_client<std_srvs::srv::Trigger>(pair.second);
  scaling_service_=node_->create_service<std_srvs::srv::Trigger>("/soarm4/rviz_scaling",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
   auto read=[this,response](){
    QDoubleSpinBox *velocity=nullptr,*acceleration=nullptr;int nv=0,na=0;
    for(auto* widget:QApplication::allWidgets()){
     if(widget->window()!=this->window())continue;
     auto* spin=qobject_cast<QDoubleSpinBox*>(widget);if(!spin)continue;
     if(spin->objectName()=="velocity_scaling_factor"){velocity=spin;++nv;}
     if(spin->objectName()=="acceleration_scaling_factor"){acceleration=spin;++na;}
    }
    if(nv!=1||na!=1){response->success=false;response->message="MotionPlanning 속도/가속도 입력을 찾을 수 없습니다. 패널을 확인하세요.";return;}
    velocity->interpretText();acceleration->interpretText();
    QJsonObject values{{"velocity",velocity->value()},{"acceleration",acceleration->value()}};
    response->success=true;response->message=QJsonDocument(values).toJson(QJsonDocument::Compact).toStdString();
   };
   if(QThread::currentThread()==thread())read();else QMetaObject::invokeMethod(this,read,Qt::BlockingQueuedConnection);
  });
  // Apply the requested demo ratios once, after MotionPlanning creates its widgets.
  // These are planner ratios in (0,1]; joint/servo limits are unchanged.
  const double demo_velocity=node_->declare_parameter<double>("demo_velocity_scaling",0.80);
  const double demo_acceleration=node_->declare_parameter<double>("demo_acceleration_scaling",0.40);
  if(!(demo_velocity>0.0&&demo_velocity<=1.0&&demo_acceleration>0.0&&demo_acceleration<=1.0))
    throw std::runtime_error("Demo scaling must be in (0,1]");
  auto* scaling_timer=new QTimer(this);
  connect(scaling_timer,&QTimer::timeout,this,[this,scaling_timer,demo_velocity,demo_acceleration](){
   QDoubleSpinBox *velocity=nullptr,*acceleration=nullptr;int nv=0,na=0;
   for(auto* widget:QApplication::allWidgets()){
    if(widget->window()!=this->window())continue;
    auto* spin=qobject_cast<QDoubleSpinBox*>(widget);if(!spin)continue;
    if(spin->objectName()=="velocity_scaling_factor"){velocity=spin;++nv;}
    if(spin->objectName()=="acceleration_scaling_factor"){acceleration=spin;++na;}
   }
   if(nv!=1||na!=1)return;
   velocity->setValue(demo_velocity);acceleration->setValue(demo_acceleration);
   RCLCPP_INFO(node_->get_logger(),"Demo ratios applied: velocity=%.2f acceleration=%.2f; limits unchanged",velocity->value(),acceleration->value());
   scaling_timer->stop();scaling_timer->deleteLater();
  });
  scaling_timer->start(200);
  edit_pub_=node_->create_publisher<std_msgs::msg::String>("/soarm4/pick_place/edit",10);QPointer<HomePanel> guard(this);
  diff_sub_=node_->create_subscription<std_msgs::msg::Int32>("/vacuum/diff",10,[guard](std_msgs::msg::Int32::ConstSharedPtr m){if(!guard)return;int value=m->data;QMetaObject::invokeMethod(guard,[guard,value](){if(guard){guard->diff_=value;guard->have_pressure_=true;guard->pressure_elapsed_.start();}},Qt::QueuedConnection);});
  hold_sub_=node_->create_subscription<std_msgs::msg::Bool>("/vacuum/holding",10,[guard](std_msgs::msg::Bool::ConstSharedPtr m){if(!guard)return;bool value=m->data;QMetaObject::invokeMethod(guard,[guard,value](){if(guard)guard->holding_=value;},Qt::QueuedConnection);});
  route_sub_=node_->create_subscription<std_msgs::msg::String>("/soarm4/pick_place/route",rclcpp::QoS(1).transient_local(),[guard](std_msgs::msg::String::ConstSharedPtr m){if(!guard)return;QString text=QString::fromStdString(m->data);QMetaObject::invokeMethod(guard,[guard,text](){if(guard)guard->setRoute(text);},Qt::QueuedConnection);});
  edit_sub_=node_->create_subscription<std_msgs::msg::String>("/soarm4/pick_place/edit_result",10,[guard](std_msgs::msg::String::ConstSharedPtr m){if(!guard)return;QString text=QString::fromStdString(m->data);QMetaObject::invokeMethod(guard,[guard,text](){if(!guard)return;auto obj=QJsonDocument::fromJson(text.toUtf8()).object();if(obj.value("request_id").toString()!=guard->pending_)return;guard->pending_.clear();if(!obj.value("success").toBool())guard->select_index_=-1;guard->busy_=false;guard->status_->setText(obj.value("message").toString());},Qt::QueuedConnection);});
  for(const char* topic:{"/soarm4/home_status","/soarm4/pick_place_status"})status_subs_.push_back(node_->create_subscription<std_msgs::msg::String>(topic,rclcpp::QoS(1).transient_local(),[guard](std_msgs::msg::String::ConstSharedPtr m){if(!guard)return;QString text=QString::fromStdString(m->data);QMetaObject::invokeMethod(guard,[guard,text](){if(guard){guard->status_->setText(text);guard->updateTorqueLabel(text);}},Qt::QueuedConnection);}));
 }
private:
 void updateTorqueLabel(const QString& text){
  if(text.startsWith(QString::fromUtf8("토크 해제")))torque_->setText(QString::fromUtf8("현재 자세 고정 (토크 ON)"));
  else if(text.startsWith(QString::fromUtf8("현재 자세 고정"))||text.startsWith(QString::fromUtf8("홈 도착"))||text.startsWith(QString::fromUtf8("이미 홈"))||text.startsWith(QString::fromUtf8("픽앤플레이스 1회 완료")))torque_->setText(QString::fromUtf8("토크 해제 (팔 받치기)"));
 }
 QJsonObject selected() const {auto* item=route_->currentItem();return item?item->data(Qt::UserRole).toJsonObject():QJsonObject();}
 void setRoute(const QString& text){
  auto obj=QJsonDocument::fromJson(text.toUtf8()).object();if(!obj.contains("steps"))return;sensorless_=obj.contains("use_vacuum_sensor")&&!obj.value("use_vacuum_sensor").toBool();QString key=selected().value("key").toString();revision_=obj.value("revision").toInt();route_->clear();QListWidgetItem* inserted=nullptr;
  for(auto v:obj.value("steps").toArray()){auto s=v.toObject();QString label=s.value("label").toString();if(s.value("kind")=="anchor")label+=s.value("q").isArray()?QString::fromUtf8(" [저장됨]"):QString::fromUtf8(" [미저장]");auto* item=new QListWidgetItem(label,route_);item->setData(Qt::UserRole,s);if(select_index_>=0&&(s.value("kind")=="waypoint"||s.value("kind")=="action")&&s.value("section").toString()==select_section_&&s.value("index").toInt()==select_index_)inserted=item;if(s.value("key").toString()==key)route_->setCurrentItem(item);}
  if(inserted){route_->setCurrentItem(inserted);select_index_=-1;}
  if(route_->currentRow()<0&&route_->count())route_->setCurrentRow(0);
 }
 void sendEdit(QJsonObject command){
  if(busy_||!edit_pub_||!edit_pub_->get_subscription_count()){status_->setText(QString::fromUtf8("경로 편집 서비스 연결을 기다려 주세요."));return;}
  if(command.value("action").toString().startsWith("insert")){select_section_=command.value("section").toString();select_index_=command.value("index").toInt();}
  pending_=QUuid::createUuid().toString(QUuid::WithoutBraces);command.insert("request_id",pending_);command.insert("revision",revision_);std_msgs::msg::String m;m.data=QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString();busy_=true;elapsed_.start();edit_pub_->publish(m);
 }
 void saveSelected(){auto s=selected();if(s.value("kind")=="anchor")sendEdit(QJsonObject{{"action","save_anchor"},{"anchor",s.value("key")}});else if(s.value("kind")=="waypoint")editSelected("replace");else status_->setText(QString::fromUtf8("홈은 고정입니다. 이동 자세를 선택하세요."));}
 void insertAfter(const QString& action="insert"){auto s=selected();if(s.isEmpty())return;int index=s.value("index").toInt()+1;sendEdit(QJsonObject{{"action",action},{"section",s.value("section")},{"index",index}});}
 void editSelected(const QString& action,int delta=0){auto s=selected();if(s.value("kind")!="waypoint"&&s.value("kind")!="action"){status_->setText(QString::fromUtf8("이동 중간점이나 흡착/놓기 동작 단계를 선택하세요."));return;}sendEdit(QJsonObject{{"action",action},{"section",s.value("section")},{"index",s.value("index")},{"delta",delta}});}
 void moveSelected(int delta){editSelected("move",delta);}
 void request(const std::string& name){
  if(!node_||(busy_&&name!="stop"))return;auto client=clients_.at(name);if(!client->service_is_ready()){status_->setText(QString::fromUtf8("관리 서비스에 연결되지 않았습니다."));return;}if(name!="stop")busy_=true;QPointer<HomePanel> guard(this);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[guard,name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){auto response=future.get();QString text=QString::fromStdString(response->message);bool ok=response->success;if(guard)QMetaObject::invokeMethod(guard,[guard,text,ok,name](){if(!guard)return;if(name!="stop")guard->busy_=false;if(ok)guard->updateTorqueLabel(text);if(name=="list"&&ok){guard->setRoute(text);guard->status_->setText(QString::fromUtf8("저장된 경로를 불러왔습니다."));}else guard->status_->setText(text);},Qt::QueuedConnection);});
 }
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scaling_service_;
 rclcpp::Node::SharedPtr node_;std::map<std::string,rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr edit_pub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_sub_,edit_sub_;std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subs_;
 rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr diff_sub_;rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;QLabel* pressure_;QElapsedTimer pressure_elapsed_;bool sensorless_=false;bool have_pressure_=false,holding_=false;int diff_=0;
 QPushButton *pump_on_,*pump_release_,*insert_grip_,*insert_release_;QPushButton *home_,*stop_,*torque_,*save_,*add_,*up_,*down_,*remove_,*clear_,*list_,*check_,*run_;std::vector<QPushButton*> controls_;QListWidget* route_;QLabel* status_;bool busy_=false;int revision_=0;QString pending_,select_section_;int select_index_=-1;QElapsedTimer elapsed_;
};
}
PLUGINLIB_EXPORT_CLASS(soarm4_moveit_config::HomePanel,rviz_common::Panel)
