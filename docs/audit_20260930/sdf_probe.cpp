#include <sdf/sdf.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Collision.hh>
#include <sdf/Visual.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sensor.hh>
#include <iostream>
int main(int argc, char** argv) {
  sdf::Root root;
  auto errors = root.Load(argv[1]);
  std::cout << "load_errors=" << errors.size() << '\n';
  for (const auto& e : errors) std::cout << e.Message() << '\n';
  auto world = root.WorldByIndex(0);
  if (!world) {
    const auto* model = root.ModelByIndex(0);
    if (!model) return 1;
    std::cout << "model=" << model->Name() << " links=" << model->LinkCount() << '\n';
    for (unsigned i = 0; i < model->LinkCount(); ++i) {
      const auto* l = model->LinkByIndex(i);
      std::cout << " link=" << l->Name() << " pose=" << l->RawPose()
                << " sensors=" << l->SensorCount() << '\n';
      for (unsigned s = 0; s < l->SensorCount(); ++s) {
        const auto* sensor = l->SensorByIndex(s);
        std::cout << " sensor=" << sensor->Name() << " pose=" << sensor->RawPose() << '\n';
      }
    }
    return 0;
  }
  for (std::string name : {"bin_red", "bin_blue"}) {
    const auto* model = world->ModelByName(name);
    const auto* link = model->LinkByIndex(0);
    std::cout << name << " model_pose=" << model->RawPose()
              << " collisions=" << link->CollisionCount()
              << " visuals=" << link->VisualCount() << '\n';
    for (unsigned i = 0; i < link->CollisionCount(); ++i) {
      const auto* c = link->CollisionByIndex(i);
      std::cout << " collision=" << c->Name() << " pose=" << c->RawPose();
      if (auto b = c->Geom()->BoxShape()) std::cout << " box=" << b->Size();
      std::cout << '\n';
    }
    for (unsigned i = 0; i < link->VisualCount(); ++i) {
      const auto* v = link->VisualByIndex(i);
      std::cout << " visual=" << v->Name() << " pose=" << v->RawPose();
      if (auto b = v->Geom()->BoxShape()) std::cout << " box=" << b->Size();
      std::cout << '\n';
    }
  }
}
