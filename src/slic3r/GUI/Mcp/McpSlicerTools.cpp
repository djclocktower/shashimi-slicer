// MCP tools for the slicer: projects, objects, plates, presets, settings, slicing, G-code,
// printers and calibration. Each tool drives the same Plater / Tab / ObjectList entry points
// the GUI's own controls call, so the UI, undo history and dirty state follow along.
// See docs/HLSD/mcp-control.md.

#include "McpUtil.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/calib.hpp"
#include "slic3r/GUI/BackgroundSlicingProcess.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GLToolbar.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/PartPlate.hpp"
#include "slic3r/GUI/PresetComboBoxes.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/GUI/Tab.hpp"
#include "slic3r/Utils/PrintHost.hpp"

namespace Slic3r { namespace GUI { namespace Mcp {

namespace fs = boost::filesystem;

namespace {

// ---- shared helpers ------------------------------------------------------------------------

PresetBundle& bundle() { return *wxGetApp().preset_bundle; }

ObjectList& object_list()
{
    ObjectList* l = wxGetApp().obj_list();
    if (l == nullptr)
        throw ToolError("object list not ready", -32001);
    return *l;
}

Tab& tab_of(Preset::Type type)
{
    Tab* tab = wxGetApp().get_tab(type);
    if (tab == nullptr)
        throw ToolError("settings tab not ready", -32001);
    return *tab;
}

PresetCollection& collection_of(Preset::Type type)
{
    switch (type) {
    case Preset::TYPE_PRINT:    return bundle().prints;
    case Preset::TYPE_FILAMENT: return bundle().filaments;
    default:                    return bundle().printers;
    }
}

PartPlate& plate_at(int idx)
{
    PartPlateList& plates = plater().get_partplate_list();
    if (idx < 0)
        idx = plates.get_curr_plate_index();
    if (idx >= plates.get_plate_count())
        throw ToolError("no plate " + std::to_string(idx) + " (there are " + std::to_string(plates.get_plate_count()) + ")", -32602);
    return *plates.get_plate(idx);
}

// The value of a JSON parameter as the config's own serialisation ("0.2", "1", "a;b").
std::string config_string(const json& v)
{
    if (v.is_string())  return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "1" : "0";
    if (v.is_number())  return v.dump();
    if (v.is_array()) {
        std::string out;
        for (const json& e : v) {
            if (!out.empty())
                out += ",";
            out += config_string(e);
        }
        return out;
    }
    throw ToolError("setting values must be strings, numbers, booleans or arrays", -32602);
}

json config_json(const DynamicPrintConfig& cfg)
{
    json out = json::object();
    for (const std::string& key : cfg.keys())
        out[key] = cfg.opt_serialize(key);
    return out;
}

// Which preset collection's edited config holds `key`; TYPE_INVALID when none does.
Preset::Type owner_of(const std::string& key)
{
    for (Preset::Type t : {Preset::TYPE_PRINT, Preset::TYPE_FILAMENT, Preset::TYPE_PRINTER})
        if (collection_of(t).get_edited_preset().config.has(key))
            return t;
    return Preset::TYPE_INVALID;
}

void select_objects(const std::vector<int>& objects, int instance = -1)
{
    Plater&    p   = plater();
    Selection& sel = p.canvas3D()->get_selection();
    sel.clear();
    for (int idx : objects)
        if (instance >= 0)
            sel.add_instance(unsigned(idx), unsigned(instance), false);
        else
            sel.add_object(unsigned(idx), false);
    p.canvas3D()->set_as_dirty();
    object_list().update_selections();
}

// Rebuild the object list from the model after a change the list does not hear about.
void refresh_object_list() { object_list().update_after_undo_redo(); }

const char* volume_type_name(ModelVolumeType t)
{
    switch (t) {
    case ModelVolumeType::MODEL_PART:         return "part";
    case ModelVolumeType::NEGATIVE_VOLUME:    return "negative";
    case ModelVolumeType::PARAMETER_MODIFIER: return "modifier";
    case ModelVolumeType::SUPPORT_BLOCKER:    return "support_blocker";
    case ModelVolumeType::SUPPORT_ENFORCER:   return "support_enforcer";
    default:                                  return "other";
    }
}

ModelVolumeType volume_type(const std::string& name)
{
    if (name == "part")             return ModelVolumeType::MODEL_PART;
    if (name == "negative")         return ModelVolumeType::NEGATIVE_VOLUME;
    if (name == "modifier")         return ModelVolumeType::PARAMETER_MODIFIER;
    if (name == "support_blocker")  return ModelVolumeType::SUPPORT_BLOCKER;
    if (name == "support_enforcer") return ModelVolumeType::SUPPORT_ENFORCER;
    throw ToolError("volume type must be part, negative, modifier, support_blocker or support_enforcer", -32602);
}

const json k_volume_types = json::array({"part", "negative", "modifier", "support_blocker", "support_enforcer"});

double deg(double rad) { return rad * 180. / PI; }

json bbox_json(const BoundingBoxf3& bb)
{
    if (!bb.defined)
        return json();
    return json{{"min", vec3(bb.min)}, {"max", vec3(bb.max)}, {"size", vec3(bb.size())}};
}

// ---- project -------------------------------------------------------------------------------

void require_discardable(bool discard)
{
    if (!discard && plater().is_project_dirty())
        throw ToolError("the project has unsaved changes: save it with project_save, or pass discard_changes: true", -32010);
}

json project_new(const json& params)
{
    const bool discard = arg<bool>(params, "discard_changes", false);
    require_discardable(discard);
    plater().new_project(true, true);
    return json{{"objects", plater().model().objects.size()}};
}

json project_open(const json& params)
{
    const std::string path = req<std::string>(params, "path");
    if (!fs::exists(fs::path(path)))
        throw ToolError("no such file: " + path, -32602);
    const bool discard = arg<bool>(params, "discard_changes", false);
    require_discardable(discard);
    if (discard)
        plater().reset_project_dirty_after_save();
    plater().load_project(from_utf8(path));
    return json{{"project", utf8(plater().get_project_name())}, {"objects", plater().model().objects.size()},
                {"plates", plater().get_partplate_list().get_plate_count()}};
}

json project_save(const json& params)
{
    Plater&           p    = plater();
    const std::string path = arg<std::string>(params, "path", "");
    if (!path.empty())
        p.set_project_filename(from_utf8(path));
    else if (p.get_project_filename(".3mf").empty())
        throw ToolError("the project has never been saved: pass a path", -32602);
    if (p.save_project(false) != wxID_YES)
        throw ToolError("saving the project failed");
    return json{{"path", utf8(p.get_project_filename(".3mf"))}};
}

json project_import(const json& params)
{
    std::vector<fs::path> files;
    for (const std::string& f : req<std::vector<std::string>>(params, "paths")) {
        if (!fs::exists(fs::path(f)))
            throw ToolError("no such file: " + f, -32602);
        files.emplace_back(f);
    }
    if (files.empty())
        throw ToolError("paths is empty", -32602);
    LoadStrategy strategy = LoadStrategy::LoadModel;
    if (arg<bool>(params, "load_settings", false))
        strategy = strategy | LoadStrategy::LoadConfig;
    const std::vector<size_t> added = plater().load_files(files, strategy, false);
    return json{{"added_objects", added}, {"objects", plater().model().objects.size()}};
}

json project_export_3mf(const json& params)
{
    const std::string path  = req<std::string>(params, "path");
    const int         plate = arg<int>(params, "plate", -1);
    if (plate >= plater().get_partplate_list().get_plate_count())
        throw ToolError("no plate " + std::to_string(plate), -32602);
    const int rc = plater().export_3mf(fs::path(path), SaveStrategy::SplitModel | SaveStrategy::ShareMesh, plate);
    if (rc < 0)
        throw ToolError("3MF export failed (" + std::to_string(rc) + ")");
    return json{{"path", path}};
}

json model_export(const json& params)
{
    const std::string path   = req<std::string>(params, "path");
    const std::string format = arg<std::string>(params, "format", boost::iends_with(path, ".obj") ? "obj" : "stl");
    Model&            model  = plater().model();
    std::vector<int>  objects;
    if (params.contains("objects"))
        objects = object_indices(params);
    else
        for (int i = 0; i < int(model.objects.size()); ++i)
            objects.push_back(i);
    if (objects.empty())
        throw ToolError("nothing to export", -32602);
    TriangleMesh mesh;
    for (int idx : objects)
        mesh.merge(Plater::combine_mesh_fff(*model.objects[idx], -1));   // all instances, world space
    const bool ok = format == "obj" ? store_obj(path.c_str(), &mesh) : store_stl(path.c_str(), &mesh, true);
    if (!ok)
        throw ToolError("cannot write " + path);
    return json{{"path", path}, {"triangles", mesh.facets_count()}};
}

json load_gcode(const json& params)
{
    const std::string path = req<std::string>(params, "path");
    if (!fs::exists(fs::path(path)))
        throw ToolError("no such file: " + path, -32602);
    plater().load_gcode(from_utf8(path));
    return json{{"path", path}};
}

// ---- objects -------------------------------------------------------------------------------

json object_json(int idx, bool detail)
{
    Plater&        p      = plater();
    ModelObject*   o      = p.model().objects[idx];
    PartPlateList& plates = p.get_partplate_list();
    json           j{{"index", idx},
           {"name", o->name},
           {"volumes", o->volumes.size()},
           {"instances", o->instances.size()},
           {"printable", o->printable},
           {"plate", plates.find_instance(idx, 0)},
           {"bbox", bbox_json(o->instances.empty() ? BoundingBoxf3() : o->instance_bounding_box(0))},
           {"settings_overrides", o->config.size()},
           {"layer_ranges", o->layer_config_ranges.size()}};
    if (!detail)
        return j;

    json vols = json::array();
    for (size_t v = 0; v < o->volumes.size(); ++v) {
        const ModelVolume* mv = o->volumes[v];
        vols.push_back(json{{"index", v},
                            {"name", mv->name},
                            {"type", volume_type_name(mv->type())},
                            {"filament", mv->extruder_id()},
                            {"triangles", mv->mesh().facets_count()},
                            {"mesh_volume", mv->mesh().stats().volume},
                            {"offset", vec3(mv->get_offset())},
                            {"settings", config_json(mv->config.get())}});
    }
    json insts = json::array();
    for (size_t i = 0; i < o->instances.size(); ++i) {
        const ModelInstance* mi  = o->instances[i];
        const Vec3d          rot = mi->get_rotation();
        insts.push_back(json{{"index", i},
                             {"position", vec3(mi->get_offset())},
                             {"rotation", json::array({deg(rot.x()), deg(rot.y()), deg(rot.z())})},
                             {"scale", vec3(mi->get_scaling_factor())},
                             {"mirror", vec3(mi->get_mirror())},
                             {"printable", mi->printable},
                             {"plate", plates.find_instance(idx, int(i))},
                             {"bbox", bbox_json(o->instance_bounding_box(i))}});
    }
    json ranges = json::array();
    for (const auto& [range, cfg] : o->layer_config_ranges)
        ranges.push_back(json{{"range", json::array({range.first, range.second})}, {"settings", config_json(cfg.get())}});
    j["volumes"]      = std::move(vols);
    j["instances"]    = std::move(insts);
    j["settings"]     = config_json(o->config.get());
    j["layer_ranges"] = std::move(ranges);
    j["filament"]     = o->config.has("extruder") ? o->config.extruder() : 0;
    return j;
}

json objects_list(const json& params)
{
    const bool detail = arg<bool>(params, "detail", false);
    json       out    = json::array();
    for (int i = 0; i < int(plater().model().objects.size()); ++i)
        out.push_back(object_json(i, detail));
    return json{{"objects", out}};
}

json object_get(const json& params) { return object_json(object_index(params), true); }

json objects_select(const json& params)
{
    Plater& p = plater();
    if (arg<bool>(params, "all", false))
        p.select_all();
    else if (!params.contains("objects") || params["objects"].empty())
        p.deselect_all();
    else {
        const std::vector<int> objects = object_indices(params);
        const int              volume  = arg<int>(params, "volume", -1);
        if (volume >= 0) {
            if (objects.size() != 1 || volume >= int(p.model().objects[objects[0]]->volumes.size()))
                throw ToolError("volume selection needs exactly one object and a valid volume index", -32602);
            Selection& sel = p.canvas3D()->get_selection();
            sel.clear();
            sel.add_volume(unsigned(objects[0]), unsigned(volume), std::max(0, arg<int>(params, "instance", 0)), true);
            p.canvas3D()->set_as_dirty();
            object_list().update_selections();
        } else
            select_objects(objects, arg<int>(params, "instance", -1));
    }
    json        sel     = json::array();
    const auto& content = p.get_selection().get_content();
    for (const auto& [obj, insts] : content)
        sel.push_back(json{{"object", obj}, {"instances", std::vector<int>(insts.begin(), insts.end())}});
    return json{{"selection", sel}};
}

json objects_delete(const json& params)
{
    Plater& p = plater();
    if (arg<bool>(params, "all", false)) {
        p.delete_all_objects_from_model();
        return json{{"objects", p.model().objects.size()}};
    }
    std::vector<int> objects = object_indices(params);
    std::sort(objects.begin(), objects.end());
    objects.erase(std::unique(objects.begin(), objects.end()), objects.end());
    // What Delete in the object list does: model and list together (it walks them backwards).
    std::vector<ItemForDelete> items;
    for (int idx : objects)
        items.emplace_back(ItemType::itObject, idx, -1);
    Plater::TakeSnapshot snapshot(&p, "Delete Objects");
    object_list().delete_from_model_and_list(items);
    return json{{"objects", p.model().objects.size()}};
}

json object_transform(const json& params)
{
    Plater&      p    = plater();
    const int    idx  = object_index(params);
    ModelObject* o    = p.model().objects[idx];
    const int    inst = arg<int>(params, "instance", 0);
    if (inst < 0 || inst >= int(o->instances.size()))
        throw ToolError("no instance " + std::to_string(inst), -32602);
    ModelInstance* mi       = o->instances[inst];
    const bool     relative = arg<bool>(params, "relative", false);

    Plater::TakeSnapshot snapshot(&p, "Transform Object");
    if (params.contains("rotation")) {
        const Vec3d r = to_vec3(params, "rotation") * (PI / 180.);
        mi->set_rotation(relative ? Vec3d(mi->get_rotation() + r) : r);
    }
    if (params.contains("scale")) {
        const Vec3d s = to_vec3(params, "scale");
        if (s.minCoeff() <= 0.)
            throw ToolError("scale factors must be positive", -32602);
        mi->set_scaling_factor(relative ? Vec3d(mi->get_scaling_factor().cwiseProduct(s)) : s);
    }
    if (params.contains("size")) {
        // Target world bounding-box size; a component <= 0 keeps that axis. Exact when the
        // rotation is a multiple of 90 degrees about each axis.
        o->invalidate_bounding_box();
        const Vec3d want = to_vec3(params, "size");
        const Vec3d have = o->instance_bounding_box(inst).size();
        Vec3d       f    = Vec3d::Ones();
        for (int a = 0; a < 3; ++a)
            if (want[a] > 0. && have[a] > EPSILON)
                f[a] = want[a] / have[a];
        if (arg<bool>(params, "uniform", false)) {
            const double u = want.maxCoeff() > 0. ? f[want.x() > 0. ? 0 : want.y() > 0. ? 1 : 2] : 1.;
            f              = Vec3d(u, u, u);
        }
        mi->set_scaling_factor(mi->get_scaling_factor().cwiseProduct(f));
    }
    if (params.contains("position")) {
        const Vec3d v = to_vec3(params, "position");
        mi->set_offset(relative ? Vec3d(mi->get_offset() + v) : v);
    }
    o->invalidate_bounding_box();
    if (arg<bool>(params, "drop_to_bed", true))
        o->ensure_on_bed();
    p.get_partplate_list().notify_instance_update(idx, inst);
    p.update();
    p.get_current_canvas3D()->requires_check_outside_state();
    return object_json(idx, true)["instances"][inst];
}

json object_set(const json& params)
{
    Plater&      p   = plater();
    const int    idx = object_index(params);
    ModelObject* o   = p.model().objects[idx];
    Plater::TakeSnapshot snapshot(&p, "Edit Object");
    if (params.contains("name"))
        o->name = req<std::string>(params, "name");
    if (params.contains("printable")) {
        const bool v = req<bool>(params, "printable");
        o->printable = v;
        for (ModelInstance* mi : o->instances)
            mi->printable = v;
        p.canvas3D()->update_instance_printable_state_for_objects({size_t(idx)});
    }
    if (params.contains("filament")) {
        const int f = req<int>(params, "filament");   // 1-based slot, 0 = default
        if (f < 0 || f > int(bundle().filament_presets.size()))
            throw ToolError("filament must be 0 (default) or a 1-based slot up to " + std::to_string(bundle().filament_presets.size()), -32602);
        o->config.set("extruder", f);
    }
    refresh_object_list();
    p.changed_object(idx);
    return object_json(idx, false);
}

json object_instances(const json& params)
{
    Plater&   p     = plater();
    const int idx   = object_index(params);
    const int count = req<int>(params, "count");
    if (count < 1 || count > 1000)
        throw ToolError("count must be 1..1000", -32602);
    const int have = int(p.model().objects[idx]->instances.size());
    select_objects({idx});
    if (count > have)
        p.increase_instances(size_t(count - have));
    else if (count < have)
        p.decrease_instances(size_t(have - count));
    return object_json(idx, false);
}

json object_mirror(const json& params)
{
    const int         idx  = object_index(params);
    const std::string axis = req<std::string>(params, "axis");
    const Axis        a    = axis == "x" ? X : axis == "y" ? Y : axis == "z" ? Z : Axis(-1);
    if (a == Axis(-1))
        throw ToolError("axis must be x, y or z", -32602);
    select_objects({idx}, arg<int>(params, "instance", -1));
    plater().mirror(a);
    return object_json(idx, true);
}

json object_split(const json& params)
{
    const int         idx  = object_index(params);
    const std::string mode = arg<std::string>(params, "mode", "objects");
    const size_t      n    = plater().model().objects.size();
    if (mode == "objects")
        plater().split_object(idx);
    else if (mode == "parts") {
        select_objects({idx});
        object_list().split();
    } else
        throw ToolError("mode must be objects or parts", -32602);
    return json{{"objects", plater().model().objects.size()}, {"added_objects", plater().model().objects.size() - n}};
}

json object_cut(const json& params)
{
    Plater&      p    = plater();
    const int    idx  = object_index(params);
    ModelObject* o    = p.model().objects[idx];
    const int    inst = arg<int>(params, "instance", 0);
    if (inst < 0 || inst >= int(o->instances.size()))
        throw ToolError("no instance " + std::to_string(inst), -32602);
    const double      z    = req<double>(params, "z");
    const std::string keep = arg<std::string>(params, "keep", "both");

    ModelObjectCutAttributes attrs;
    if (keep == "both" || keep == "upper") attrs = attrs | ModelObjectCutAttribute::KeepUpper;
    if (keep == "both" || keep == "lower") attrs = attrs | ModelObjectCutAttribute::KeepLower;
    if (!attrs.has(ModelObjectCutAttribute::KeepUpper) && !attrs.has(ModelObjectCutAttribute::KeepLower))
        throw ToolError("keep must be both, upper or lower", -32602);
    if (arg<bool>(params, "keep_as_parts", false)) attrs = attrs | ModelObjectCutAttribute::KeepAsParts;
    if (arg<bool>(params, "flip_upper", false))    attrs = attrs | ModelObjectCutAttribute::FlipUpper;
    if (arg<bool>(params, "place_on_cut", false)) {
        attrs = attrs | ModelObjectCutAttribute::PlaceOnCutUpper;
        attrs = attrs | ModelObjectCutAttribute::PlaceOnCutLower;
    }

    // Same construction as Plater::cut_horizontal (the calibration cuts).
    Plater::TakeSnapshot snapshot(&p, "Cut by Plane");
    const Vec3d  offset = o->instances[inst]->get_offset();
    Cut          cut(o, inst, Geometry::translation_transform(z * Vec3d::UnitZ() - offset), attrs);
    const auto   parts  = cut.perform_with_plane();
    p.apply_cut_object_to_model(size_t(idx), parts);
    return json{{"objects", p.model().objects.size()}};
}

json object_add_shape(const json& params)
{
    Plater&           p     = plater();
    const std::string shape = req<std::string>(params, "shape");
    static const std::set<std::string> shapes{"Cube", "Cylinder", "Sphere", "Cone", "Disc", "Torus"};
    if (!shapes.count(shape))
        throw ToolError("shape must be Cube, Cylinder, Sphere, Cone, Disc or Torus", -32602);
    const std::string as = arg<std::string>(params, "as", "object");
    if (as == "object") {
        p.deselect_all();
        object_list().load_shape_object(shape);
        return object_json(int(p.model().objects.size()) - 1, false);
    }
    const int idx = object_index(params);
    select_objects({idx});
    object_list().load_generic_subobject(shape, volume_type(as));
    return object_json(idx, true);
}

json volume_set(const json& params)
{
    Plater&      p   = plater();
    const int    idx = object_index(params);
    ModelObject* o   = p.model().objects[idx];
    const int    v   = req<int>(params, "volume");
    if (v < 0 || v >= int(o->volumes.size()))
        throw ToolError("no volume " + std::to_string(v), -32602);
    ModelVolume* mv = o->volumes[v];
    if (params.contains("type")) {
        Selection& sel = p.canvas3D()->get_selection();
        sel.clear();
        sel.add_volume(unsigned(idx), unsigned(v), 0, true);
        object_list().update_selections();
        object_list().set_volume_type(volume_type(req<std::string>(params, "type")));
    }
    if (params.contains("name") || params.contains("filament")) {
        Plater::TakeSnapshot snapshot(&p, "Edit Part");
        if (params.contains("name"))
            mv->name = req<std::string>(params, "name");
        if (params.contains("filament")) {
            const int f = req<int>(params, "filament");
            if (f < 0 || f > int(bundle().filament_presets.size()))
                throw ToolError("filament must be 0 (object's) or a 1-based slot", -32602);
            mv->config.set("extruder", f);
        }
        refresh_object_list();
        p.changed_object(idx);
    }
    return object_json(idx, true)["volumes"][v];
}

json volume_delete(const json& params)
{
    const int idx = object_index(params);
    const int v   = req<int>(params, "volume");
    if (v < 0 || v >= int(plater().model().objects[idx]->volumes.size()))
        throw ToolError("no volume " + std::to_string(v), -32602);
    if (plater().model().objects[idx]->volumes.size() == 1)
        throw ToolError("an object's last part cannot be deleted; delete the object", -32602);
    object_list().delete_from_model_and_list(ItemType::itVolume, idx, v);
    return object_json(idx, true);
}

// ---- per-object settings -------------------------------------------------------------------

struct SettingsTarget
{
    ModelConfig* config = nullptr;
    bool         volume = false;
    bool         layer  = false;
};

SettingsTarget settings_target(const json& params, bool create)
{
    ModelObject* o = plater().model().objects[object_index(params)];
    if (params.contains("volume")) {
        const int v = req<int>(params, "volume");
        if (v < 0 || v >= int(o->volumes.size()))
            throw ToolError("no volume " + std::to_string(v), -32602);
        return {&o->volumes[v]->config, true, false};
    }
    if (params.contains("layer_range")) {
        const auto r = req<std::vector<double>>(params, "layer_range");
        if (r.size() != 2 || !(r[0] < r[1]) || r[0] < 0.)
            throw ToolError("layer_range must be [min_z, max_z] with min_z < max_z", -32602);
        const t_layer_height_range range{r[0], r[1]};
        auto                       it = o->layer_config_ranges.find(range);
        if (it == o->layer_config_ranges.end()) {
            if (!create)
                throw ToolError("no layer range [" + std::to_string(r[0]) + ", " + std::to_string(r[1]) + "]", -32602);
            // The same starting point the object list gives a new range.
            ModelConfig& cfg = o->layer_config_ranges[range];
            cfg.assign_config(object_list().get_default_layer_config(object_index(params)));
            return {&cfg, false, true};
        }
        return {&it->second, false, true};
    }
    return {&o->config, false, false};
}

void check_settable(const std::string& key, const SettingsTarget& t)
{
    static const PrintObjectConfig object_keys;
    static const PrintRegionConfig region_keys;
    const bool region = region_keys.has(key) || key == "extruder";
    const bool object = object_keys.has(key);
    if (t.volume ? region : t.layer ? (region || key == "layer_height") : (region || object))
        return;
    throw ToolError("'" + key + "' cannot be set " + (t.volume ? "on a part" : t.layer ? "on a layer range" : "on an object") +
                        " (only per-object / per-region print settings can; see config_describe)", -32602);
}

json object_settings_get(const json& params)
{
    SettingsTarget t = settings_target(params, false);
    return json{{"settings", config_json(t.config->get())}};
}

json object_settings_set(const json& params)
{
    Plater&   p   = plater();
    const int idx = object_index(params);
    Plater::TakeSnapshot snapshot(&p, "Change Object Settings");
    if (params.contains("layer_range") && arg<bool>(params, "remove_range", false)) {
        const auto r = req<std::vector<double>>(params, "layer_range");
        if (r.size() != 2 || p.model().objects[idx]->layer_config_ranges.erase(t_layer_height_range{r[0], r[1]}) == 0)
            throw ToolError("no such layer range", -32602);
        refresh_object_list();
        p.changed_object(idx);
        return json{{"layer_ranges", object_json(idx, true)["layer_ranges"]}};
    }
    {
        SettingsTarget t = settings_target(params, true);
        const json     settings = arg<json>(params, "settings", json::object());
        if (!settings.is_object())
            throw ToolError("settings must be an object {key: value}", -32602);
        for (auto it = settings.begin(); it != settings.end(); ++it) {
            check_settable(it.key(), t);
            ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Disable);
            try {
                t.config->set_deserialize(it.key(), config_string(it.value()), ctx);
            } catch (const std::exception& ex) {
                throw ToolError("invalid value for '" + it.key() + "': " + ex.what(), -32602);
            }
        }
        for (const std::string& key : arg<std::vector<std::string>>(params, "reset", {}))
            t.config->erase(key);
    }
    refresh_object_list();
    p.changed_object(idx);
    return object_settings_get(params);
}

// ---- plates --------------------------------------------------------------------------------

json plate_json(int i)
{
    Plater&        p      = plater();
    PartPlateList& plates = p.get_partplate_list();
    PartPlate&     plate  = *plates.get_plate(i);
    json           objs   = json::array();
    for (int o = 0; o < int(p.model().objects.size()); ++o)
        for (int k = 0; k < int(p.model().objects[o]->instances.size()); ++k)
            if (plates.find_instance(o, k) == i)
                objs.push_back(json{{"object", o}, {"instance", k}});
    return json{{"index", i},
                {"name", plate.get_plate_name()},
                {"current", i == plates.get_curr_plate_index()},
                {"locked", plate.is_locked()},
                {"bed_type", plate.get_bed_type() == btDefault ? std::string("default") : ConfigOptionEnum<BedType>(plate.get_bed_type()).serialize()},
                {"print_sequence", plate.get_print_seq() == PrintSequence::ByDefault ? std::string("default") : ConfigOptionEnum<PrintSequence>(plate.get_print_seq()).serialize()},
                {"spiral_vase", plate.has_spiral_mode_config() ? json(plate.get_spiral_vase_mode()) : json("default")},
                {"origin", vec3(plate.get_origin())},
                {"objects", objs},
                {"sliced", plate.is_slice_result_valid()},
                {"slicing_percent", plate.get_slicing_percent()},
                {"error", p.last_slicing_error(i)}};
}

json plates_list(const json&)
{
    json out = json::array();
    for (int i = 0; i < plater().get_partplate_list().get_plate_count(); ++i)
        out.push_back(plate_json(i));
    return json{{"plates", out}};
}

int plate_param(const json& params)
{
    const int i = req<int>(params, "plate");
    plate_at(i);
    return i;
}

json plate_select(const json& params)
{
    const int i = plate_param(params);
    plater().select_plate(i);
    return plate_json(i);
}

json plate_add(const json&)
{
    if (!plater().can_add_plate())
        throw ToolError("no more plates can be added");
    plater().add_plate();
    return plates_list(json());
}

json plate_delete(const json& params)
{
    if (!plater().can_delete_plate())
        throw ToolError("the last plate cannot be deleted");
    plater().delete_plate(plate_param(params));
    return plates_list(json());
}

json plate_duplicate(const json& params)
{
    if (!plater().can_add_plate())
        throw ToolError("no more plates can be added");
    plater().duplicate_plate(plate_param(params));
    return plates_list(json());
}

json plate_set(const json& params)
{
    const int  i     = plate_param(params);
    PartPlate& plate = plate_at(i);
    if (params.contains("name"))
        plate.set_plate_name(req<std::string>(params, "name"));
    if (params.contains("locked"))
        plater().get_partplate_list().lock_plate(i, req<bool>(params, "locked"));
    if (params.contains("bed_type")) {
        const std::string s  = req<std::string>(params, "bed_type");
        BedType           bt = btDefault;
        if (s != "default" && !ConfigOptionEnum<BedType>::from_string(s, bt))
            throw ToolError("unknown bed_type '" + s + "' (see config_describe key=curr_bed_type)", -32602);
        plate.set_bed_type(bt);
    }
    if (params.contains("print_sequence")) {
        const std::string s   = req<std::string>(params, "print_sequence");
        PrintSequence     seq = PrintSequence::ByDefault;
        if (s != "default" && !ConfigOptionEnum<PrintSequence>::from_string(s, seq))
            throw ToolError("print_sequence must be default, 'by layer' or 'by object'", -32602);
        plate.set_print_seq(seq);
    }
    if (params.contains("spiral_vase"))
        plate.set_spiral_vase_mode(req<bool>(params, "spiral_vase"), false);
    plate.update_slice_result_valid_state(false);
    plater().update();
    return plate_json(i);
}

json object_move_to_plate(const json& params)
{
    Plater&        p      = plater();
    const int      idx    = object_index(params);
    const int      inst   = arg<int>(params, "instance", 0);
    ModelObject*   o      = p.model().objects[idx];
    if (inst < 0 || inst >= int(o->instances.size()))
        throw ToolError("no instance " + std::to_string(inst), -32602);
    PartPlate&     to     = plate_at(plate_param(params));
    PartPlateList& plates = p.get_partplate_list();
    const int      from_i = plates.find_instance(idx, inst);
    ModelInstance* mi     = o->instances[inst];

    Plater::TakeSnapshot snapshot(&p, "Move Object to Plate");
    const Vec3d off = mi->get_offset();
    Vec3d       pos;
    if (from_i >= 0)   // keep its place relative to the plate
        pos = to.get_origin() + (off - plates.get_plate(from_i)->get_origin());
    else
        pos = to.get_center_origin();
    pos.z() = off.z();
    mi->set_offset(pos);
    o->invalidate_bounding_box();
    plates.notify_instance_update(idx, inst);
    p.update();
    return object_json(idx, true)["instances"][inst];
}

// ---- presets -------------------------------------------------------------------------------

json presets_list(const json& params)
{
    const Preset::Type t                    = preset_type(req<std::string>(params, "type"));
    PresetCollection&  c                    = collection_of(t);
    const bool         include_incompatible = arg<bool>(params, "include_incompatible", false);
    const bool         include_hidden       = arg<bool>(params, "include_hidden", false);
    const std::string  selected             = c.get_selected_preset_name();
    json               out                  = json::array();
    for (const Preset& pr : c.get_presets()) {
        if (pr.is_default && pr.name != selected)
            continue;
        if (!include_hidden && !pr.is_visible)
            continue;
        if (!include_incompatible && !pr.is_compatible)
            continue;
        out.push_back(json{{"name", pr.name},
                           {"system", pr.is_system},
                           {"user", pr.is_user()},
                           {"project", pr.is_project_embedded},
                           {"compatible", pr.is_compatible},
                           {"inherits", pr.inherits()},
                           {"selected", pr.name == selected}});
    }
    json res{{"type", preset_type_name(t)}, {"selected", selected}, {"dirty", c.current_is_dirty()}, {"presets", out}};
    if (t == Preset::TYPE_FILAMENT)
        res["slots"] = bundle().filament_presets;
    return res;
}

json preset_select(const json& params)
{
    Plater&            p    = plater();
    const Preset::Type t    = preset_type(req<std::string>(params, "type"));
    const std::string  name = req<std::string>(params, "name");
    PresetCollection&  c    = collection_of(t);
    if (c.find_preset(name, false) == nullptr)
        throw ToolError("no " + preset_type_name(t) + " preset named '" + name + "' (see presets_list)", -32602);
    Tab& tab = tab_of(t);

    if (t == Preset::TYPE_FILAMENT) {
        const int slot = arg<int>(params, "slot", 0);
        if (slot < 0 || slot >= int(bundle().filament_presets.size()))
            throw ToolError("no filament slot " + std::to_string(slot) + " (0-based; there are " +
                                std::to_string(bundle().filament_presets.size()) + ")", -32602);
        // What the sidebar's filament combo does on a selection (Plater::priv::on_select_preset).
        bundle().set_filament_preset(size_t(slot), name);
        p.update_project_dirty_from_presets();
        bundle().export_selections(*wxGetApp().app_config);
        p.sidebar().update_dynamic_filament_list();
        p.on_filament_change(size_t(slot));
        if (!p.sidebar().is_multifilament()) {
            tab.select_preset(name);
            p.on_config_change(bundle().full_config());
        } else if (size_t(slot) < p.sidebar().combos_filament().size())
            p.sidebar().combos_filament()[slot]->update();
        return presets_list(json{{"type", "filament"}})["slots"];
    }

    if (c.current_is_dirty()) {
        if (!arg<bool>(params, "discard_changes", false))
            throw ToolError("the " + preset_type_name(t) + " preset has unsaved changes: preset_save them, or pass discard_changes: true", -32010);
        c.discard_current_changes();
    }
    auto select = [&]() {
        tab.select_preset(name);
        p.on_config_change(bundle().full_config());
    };
    if (t == Preset::TYPE_PRINTER) {
        // A preset outside the selected physical printer's set ends that selection, as in the combo.
        PhysicalPrinterCollection& pp = bundle().physical_printers;
        if (pp.has_selection() && pp.get_selected_printer().preset_names.count(name) == 0)
            pp.unselect_printer();
        p.update_objects_position_when_select_preset(select);
    } else
        select();
    return json{{"type", preset_type_name(t)}, {"selected", c.get_selected_preset_name()}};
}

json preset_save(const json& params)
{
    const Preset::Type t    = preset_type(req<std::string>(params, "type"));
    PresetCollection&  c    = collection_of(t);
    std::string        name = arg<std::string>(params, "name", "");
    if (name.empty()) {
        const Preset& sel = c.get_selected_preset();
        if (sel.is_system || sel.is_default)
            throw ToolError("the selected preset is a system preset: pass a new name to save a user preset", -32602);
        name = sel.name;
    }
    if (const Preset* existing = c.find_preset(name, false); existing && (existing->is_system || existing->is_default))
        throw ToolError("'" + name + "' is a system preset and cannot be overwritten", -32602);
    tab_of(t).save_preset(name, arg<bool>(params, "detach", false));
    if (c.get_selected_preset_name() != name)
        throw ToolError("saving the preset did not complete (check validation messages with ui_windows)");
    return json{{"type", preset_type_name(t)}, {"saved", name}, {"dirty", c.current_is_dirty()}};
}

json preset_discard(const json& params)
{
    const Preset::Type t = preset_type(req<std::string>(params, "type"));
    collection_of(t).discard_current_changes();
    Tab& tab = tab_of(t);
    tab.load_current_preset();
    tab.update_dirty();
    plater().on_config_change(bundle().full_config());
    return json{{"type", preset_type_name(t)}, {"dirty", collection_of(t).current_is_dirty()}};
}

json preset_diff(const json& params)
{
    const Preset::Type t = preset_type(req<std::string>(params, "type"));
    PresetCollection&  c = collection_of(t);
    json               out = json::object();
    for (const std::string& key : c.current_dirty_options())
        out[key] = json{{"edited", c.get_edited_preset().config.opt_serialize(key)},
                        {"saved", c.get_selected_preset().config.has(key) ? c.get_selected_preset().config.opt_serialize(key) : std::string()}};
    return json{{"type", preset_type_name(t)}, {"preset", c.get_selected_preset_name()}, {"changes", out}};
}

json filament_add(const json& params)
{
    Plater& p = plater();
    p.sidebar().add_filament();
    const int slot = int(bundle().filament_presets.size()) - 1;
    if (const std::string preset = arg<std::string>(params, "preset", ""); !preset.empty())
        preset_select(json{{"type", "filament"}, {"name", preset}, {"slot", slot}});
    return json{{"slot", slot}, {"slots", bundle().filament_presets}};
}

json filament_remove(const json& params)
{
    const int slot = req<int>(params, "slot");
    if (slot < 0 || slot >= int(bundle().filament_presets.size()))
        throw ToolError("no filament slot " + std::to_string(slot), -32602);
    if (bundle().filament_presets.size() == 1)
        throw ToolError("the last filament cannot be removed", -32602);
    plater().sidebar().delete_filament(size_t(slot), arg<int>(params, "replace_with", -1));
    return json{{"slots", bundle().filament_presets}};
}

// ---- settings ------------------------------------------------------------------------------

const char* option_type_name(ConfigOptionType t)
{
    switch (t) {
    case coFloat:          return "float";
    case coFloats:         return "floats";
    case coInt:            return "int";
    case coInts:           return "ints";
    case coString:         return "string";
    case coStrings:        return "strings";
    case coPercent:        return "percent";
    case coPercents:       return "percents";
    case coFloatOrPercent: return "float_or_percent";
    case coFloatsOrPercents: return "floats_or_percents";
    case coPoint:          return "point";
    case coPoints:         return "points";
    case coPoint3:         return "point3";
    case coBool:           return "bool";
    case coBools:          return "bools";
    case coEnum:           return "enum";
    case coEnums:          return "enums";
    default:               return "other";
    }
}

std::string owner_name(const std::string& key)
{
    const Preset::Type t = owner_of(key);
    if (t != Preset::TYPE_INVALID)
        return preset_type_name(t);
    if (bundle().project_config.has(key))
        return "project";
    return "object_only";
}

json config_describe(const json& params)
{
    const std::string key    = arg<std::string>(params, "key", "");
    const std::string search = boost::to_lower_copy(arg<std::string>(params, "search", ""));
    const std::string cat    = arg<std::string>(params, "category", "");
    const size_t      limit  = size_t(std::clamp(arg<int>(params, "limit", 50), 1, 2000));
    const DynamicPrintConfig full = bundle().full_config();

    auto describe = [&](const std::string& k, const ConfigOptionDef& d) {
        json j{{"key", k},
               {"label", d.label},
               {"full_label", d.full_label},
               {"category", d.category},
               {"type", option_type_name(d.type)},
               {"owner", owner_name(k)},
               {"mode", d.mode == comSimple ? "simple" : d.mode == comAdvanced ? "advanced" : "expert"},
               {"tooltip", d.tooltip}};
        if (!d.sidetext.empty())      j["unit"] = d.sidetext;
        if (d.min > -FLT_MAX)         j["min"] = d.min;
        if (d.max < FLT_MAX)          j["max"] = d.max;
        if (!d.enum_values.empty())   j["enum_values"] = d.enum_values;
        if (!d.enum_labels.empty())   j["enum_labels"] = d.enum_labels;
        if (d.default_value)          j["default"] = d.default_value->serialize();
        if (full.has(k))              j["value"] = full.opt_serialize(k);
        if (d.readonly)               j["readonly"] = true;
        return j;
    };

    if (!key.empty()) {
        const ConfigOptionDef* d = print_config_def.get(key);
        if (d == nullptr)
            throw ToolError("unknown setting '" + key + "' (use search)", -32602);
        return describe(key, *d);
    }
    json   out   = json::array();
    size_t total = 0;
    for (const auto& [k, d] : print_config_def.options) {
        if (!cat.empty() && !boost::iequals(d.category, cat))
            continue;
        if (!search.empty() && !boost::icontains(k, search) && !boost::icontains(d.label, search) &&
            !boost::icontains(d.full_label, search) && !boost::icontains(d.tooltip, search))
            continue;
        if (++total <= limit)
            out.push_back(describe(k, d));
    }
    return json{{"total", total}, {"options", out}};
}

json config_get(const json& params)
{
    const DynamicPrintConfig full = bundle().full_config();
    json                     out  = json::object();
    if (params.contains("keys")) {
        for (const std::string& key : req<std::vector<std::string>>(params, "keys")) {
            if (!full.has(key))
                throw ToolError("unknown setting '" + key + "' (see config_describe)", -32602);
            const Preset::Type t     = owner_of(key);
            bool               dirty = false;
            if (t != Preset::TYPE_INVALID) {
                const auto opts = collection_of(t).current_dirty_options();
                dirty           = std::find(opts.begin(), opts.end(), key) != opts.end();
            }
            out[key] = json{{"value", full.opt_serialize(key)}, {"owner", owner_name(key)}, {"modified", dirty}};
        }
        return json{{"settings", out}};
    }
    if (params.contains("type")) {
        const Preset::Type t = preset_type(req<std::string>(params, "type"));
        return json{{"type", preset_type_name(t)}, {"preset", collection_of(t).get_edited_preset().name},
                    {"settings", config_json(collection_of(t).get_edited_preset().config)}};
    }
    throw ToolError("pass keys: [...] or type: print|filament|printer", -32602);
}

json config_set(const json& params)
{
    const json settings = req<json>(params, "settings");
    if (!settings.is_object() || settings.empty())
        throw ToolError("settings must be a non-empty object {key: value}", -32602);

    // Group by the preset that owns each key, validating everything before changing anything.
    // A few keys (compatible_printers, inherits...) live in more than one preset; `type` picks one.
    const std::string forced = arg<std::string>(params, "type", "");
    std::map<Preset::Type, DynamicPrintConfig> changes;
    DynamicPrintConfig                         project_changes;
    for (auto it = settings.begin(); it != settings.end(); ++it) {
        const std::string& key = it.key();
        const Preset::Type t   = forced.empty() ? owner_of(key) : preset_type(forced);
        if (!forced.empty() && !collection_of(t).get_edited_preset().config.has(key))
            throw ToolError("'" + key + "' is not a " + forced + " setting", -32602);
        DynamicPrintConfig* target;
        if (t != Preset::TYPE_INVALID) {
            if (!changes.count(t))
                changes[t] = *tab_of(t).get_config();
            target = &changes[t];
        } else if (bundle().project_config.has(key)) {
            if (project_changes.empty())
                project_changes = bundle().project_config;
            target = &project_changes;
        } else
            throw ToolError("'" + key + "' is not a preset or project setting (per-object only? use object_settings_set; unknown? see config_describe)", -32602);
        try {
            target->set_deserialize_strict(key, config_string(it.value()));
        } catch (const std::exception& ex) {
            throw ToolError("invalid value for '" + key + "': " + ex.what(), -32602);
        }
    }

    Plater& p = plater();
    for (auto& [t, cfg] : changes) {
        // The path a typed edit takes: into the tab (marks the preset dirty, re-runs the
        // option toggling), then to the plater.
        Tab& tab = tab_of(t);
        tab.load_config(cfg);
        p.on_config_change(*tab.get_config());
    }
    if (!project_changes.empty()) {
        bundle().project_config.apply(project_changes);
        p.on_config_change(project_changes);
        p.update_project_dirty_from_presets();
    }

    json out = json::object();
    const DynamicPrintConfig full = bundle().full_config();
    for (auto it = settings.begin(); it != settings.end(); ++it)
        out[it.key()] = full.opt_serialize(it.key());
    return json{{"settings", out}};
}

// ---- slicing and G-code --------------------------------------------------------------------

json slicing_status(const json& params)
{
    Plater&        p      = plater();
    PartPlateList& plates = p.get_partplate_list();
    json           out    = json::array();
    const int      only   = arg<int>(params, "plate", -1);
    for (int i = 0; i < plates.get_plate_count(); ++i) {
        if (only >= 0 && i != only)
            continue;
        PartPlate* plate = plates.get_plate(i);
        json       j{{"plate", i},
               {"sliced", plate->is_slice_result_valid()},
               {"ready_for_print", plate->is_slice_result_ready_for_print()},
               {"has_printable_objects", plate->has_printable_instances()},
               {"percent", plate->get_slicing_percent()},
               {"error", p.last_slicing_error(i)}};
        if (plate->is_slice_result_valid())
            if (GCodeProcessorResult* r = plate->get_slice_result(); r != nullptr) {
                json warnings = json::array();
                for (const auto& w : r->warnings)
                    warnings.push_back(json{{"level", w.level}, {"message", w.msg}, {"code", w.error_code}});
                j["warnings"] = warnings;
            }
        out.push_back(j);
    }
    // "error" carries validation failures too: Plater records them where it validates, after
    // applying the model to the print. Validating here could meet a print still holding objects
    // that were deleted since the last apply.
    return json{{"busy", app_busy_state()}, {"current_plate", plates.get_curr_plate_index()}, {"plates", out}};
}

json slice(const json& params)
{
    Plater&           p     = plater();
    json              which = arg<json>(params, "plate", json("current"));
    if (which.is_string()) {
        const std::string s = which.get<std::string>();
        if (!s.empty() && s.size() < 6 && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; }))
            which = std::stoi(s);
    }
    const bool        all   = which.is_string() && which.get<std::string>() == "all";
    if (p.model().objects.empty())
        throw ToolError("nothing to slice: the project has no objects");
    if (which.is_number_integer()) {
        const int i = which.get<int>();
        plate_at(i);
        p.select_plate(i);
    } else if (!which.is_string() || (which.get<std::string>() != "current" && !all))
        throw ToolError("plate must be \"current\", \"all\" or a plate index", -32602);

    // What the Slice button does (MainFrame), minus its pre-slice dialogs.
    p.exit_gizmo();
    p.update(true, true);
    wxPostEvent(&p, SimpleEvent(all ? EVT_GLTOOLBAR_SLICE_ALL : EVT_GLTOOLBAR_SLICE_PLATE));
    if (arg<bool>(params, "show_preview", true))
        wxGetApp().mainframe->m_tabpanel->SelectPageByName(TAB_ID_PREVIEW);
    return json{{"started", true}, {"plates", all ? "all" : std::to_string(p.get_partplate_list().get_curr_plate_index())}};
}

json slice_cancel(const json&)
{
    BackgroundSlicingProcess& bg = plater().background_process();
    const bool                running = !bg.idle();
    if (running)
        bg.stop();
    return json{{"cancelled", running}};
}

json gcode_stats(const json& params)
{
    const int  i     = arg<int>(params, "plate", -1);
    PartPlate& plate = plate_at(i);
    if (!plate.is_slice_result_valid())
        throw ToolError("plate " + std::to_string(plate.get_index()) + " is not sliced (slice it first)", -32011);
    PrintBase*            print_base = nullptr;
    GCodeProcessorResult* result     = nullptr;
    int                   print_idx  = -1;
    plate.get_print(&print_base, &result, &print_idx);
    if (result == nullptr)
        result = plate.get_slice_result();
    json out{{"plate", plate.get_index()}, {"gcode_file", plate.get_tmp_gcode_path()}};
    if (result != nullptr) {
        const auto& ps   = result->print_statistics;
        const auto& norm = ps.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)];
        out["time_s"]             = norm.time;
        out["prepare_time_s"]     = norm.prepare_time;
        out["filament_changes"]   = ps.total_filament_changes;
        out["extruder_changes"]   = ps.total_extruder_changes;
        out["travel_distance_mm"] = ps.total_travel_distance;
        json per_extruder = json::object();
        for (const auto& [ext, vol] : ps.total_volumes_per_extruder)
            per_extruder[std::to_string(ext)] = vol;
        out["volume_per_filament_mm3"] = per_extruder;
        json warnings = json::array();
        for (const auto& w : result->warnings)
            warnings.push_back(json{{"level", w.level}, {"message", w.msg}, {"code", w.error_code}});
        out["warnings"] = warnings;
    }
    if (const Print* print = dynamic_cast<const Print*>(print_base); print != nullptr) {
        const PrintStatistics& st = print->print_statistics();
        out["estimated_time"]         = st.estimated_normal_print_time;
        out["filament_used_mm"]       = st.total_used_filament;
        out["filament_used_g"]        = st.total_weight;
        out["filament_cost"]          = st.total_cost;
        out["extruded_volume_mm3"]    = st.total_extruded_volume;
        out["wipe_tower_filament_mm"] = st.total_wipe_tower_filament;
        size_t layers = 0;
        for (const PrintObject* po : print->objects())
            layers = std::max(layers, po->layer_count());
        out["layers"] = layers;
    }
    return out;
}

json gcode_export(const json& params)
{
    Plater&           p    = plater();
    const std::string path = req<std::string>(params, "path");
    if (params.contains("plate")) {
        const int i = req<int>(params, "plate");
        plate_at(i);
        p.select_plate(i);
    }
    if (p.model().objects.empty())
        throw ToolError("nothing to export: the project has no objects");
    if (!p.export_gcode_to(fs::path(path)))
        throw ToolError("the export could not be started (another export running, or the plate is invalid: see slicing_status)");
    return json{{"scheduled", true}, {"path", path}};
}

json gcode_export_result(const json& params)
{
    const std::string path = req<std::string>(params, "path");
    const int         i    = plater().get_partplate_list().get_curr_plate_index();
    json              out{{"path", path}, {"exists", fs::exists(fs::path(path))}, {"error", plater().last_slicing_error(i)}};
    if (out["exists"].get<bool>())
        out["bytes"] = fs::file_size(fs::path(path));
    return out;
}

json gcode_read(const json& params)
{
    PartPlate& plate = plate_at(arg<int>(params, "plate", -1));
    std::string path = arg<std::string>(params, "path", "");
    if (path.empty()) {
        if (!plate.is_slice_result_valid())
            throw ToolError("plate " + std::to_string(plate.get_index()) + " is not sliced (slice it first, or pass a path)", -32011);
        path = plate.get_tmp_gcode_path();
    }
    std::ifstream in(fs::path(path).string());
    if (!in)
        throw ToolError("cannot read " + path);
    const long        offset    = std::max(0, arg<int>(params, "offset", 0));
    const long        max_lines = std::clamp(arg<int>(params, "max_lines", 200), 1, 5000);
    const std::string grep      = arg<std::string>(params, "grep", "");
    json              lines     = json::array();
    std::string       line;
    long              n = 0;
    while (std::getline(in, line)) {
        ++n;
        if (n <= offset)
            continue;
        if (!grep.empty() && line.find(grep) == std::string::npos)
            continue;
        lines.push_back(json{{"line", n}, {"text", line}});
        if (long(lines.size()) >= max_lines)
            break;
    }
    return json{{"path", path}, {"lines", lines}};
}

// ---- printers and calibration --------------------------------------------------------------

json printers_list(const json&)
{
    PhysicalPrinterCollection& pp  = bundle().physical_printers;
    json                       out = json::array();
    for (const PhysicalPrinter& printer : pp) {
        const DynamicPrintConfig& c = printer.config;
        out.push_back(json{{"name", printer.name},
                           {"presets", std::vector<std::string>(printer.preset_names.begin(), printer.preset_names.end())},
                           {"host_type", c.has("host_type") ? c.opt_serialize("host_type") : std::string()},
                           {"host", c.has("print_host") ? c.opt_string("print_host") : std::string()},
                           {"selected", pp.has_selection() && pp.get_selected_printer_name() == printer.name}});
    }
    const DynamicPrintConfig& cur = bundle().printers.get_edited_preset().config;
    return json{{"physical_printers", out},
                {"printer_preset_host", cur.has("print_host") ? cur.opt_string("print_host") : std::string()}};
}

json printer_upload(const json& params)
{
    const std::string path = req<std::string>(params, "path");
    if (!fs::exists(fs::path(path)))
        throw ToolError("no such file: " + path + " (export it with gcode_export first)", -32602);
    PhysicalPrinterCollection& pp     = bundle().physical_printers;
    DynamicPrintConfig*        config = nullptr;
    if (const std::string name = arg<std::string>(params, "printer", ""); !name.empty()) {
        for (PhysicalPrinter& printer : pp)
            if (printer.name == name)
                config = &printer.config;
        if (config == nullptr)
            throw ToolError("no physical printer named '" + name + "' (see printers_list)", -32602);
    } else if (pp.has_selection())
        config = &pp.get_selected_printer().config;
    else
        config = &bundle().printers.get_edited_preset().config;

    PrintHostJob job(config);
    if (!job.printhost)
        throw ToolError("no print host is configured for that printer");
    job.upload_data.source_path = fs::path(path);
    job.upload_data.upload_path = fs::path(arg<std::string>(params, "remote_name", fs::path(path).filename().string()));
    job.upload_data.group       = arg<std::string>(params, "group", "");
    job.upload_data.post_action = arg<bool>(params, "start_print", false) ? PrintHostPostUploadAction::StartPrint : PrintHostPostUploadAction::None;
    wxGetApp().printhost_job_queue().enqueue(std::move(job));
    return json{{"queued", true}, {"start_print", arg<bool>(params, "start_print", false)}};
}

json calibrate(const json& params)
{
    Plater&           p    = plater();
    const std::string test = req<std::string>(params, "test");
    Calib_Params      cp;
    cp.extruder_id         = arg<int>(params, "extruder", 0);
    cp.start               = arg<double>(params, "start", cp.start);
    cp.end                 = arg<double>(params, "end", cp.end);
    cp.step                = arg<double>(params, "step", cp.step);
    cp.print_numbers       = arg<bool>(params, "print_numbers", false);
    cp.freqStartX          = arg<double>(params, "freq_start_x", cp.freqStartX);
    cp.freqEndX            = arg<double>(params, "freq_end_x", cp.freqEndX);
    cp.freqStartY          = arg<double>(params, "freq_start_y", cp.freqStartY);
    cp.freqEndY            = arg<double>(params, "freq_end_y", cp.freqEndY);
    cp.shaper_type         = arg<std::string>(params, "shaper_type", "");
    cp.test_model          = arg<int>(params, "test_model", 0);
    cp.nozzle_based_resize = arg<bool>(params, "nozzle_based_resize", true);
    cp.accelerations       = arg<std::vector<double>>(params, "accelerations", {});
    cp.speeds              = arg<std::vector<double>>(params, "speeds", {});

    if (test == "pa_line" || test == "pa_pattern" || test == "pa_tower") {
        cp.mode = test == "pa_line" ? CalibMode::Calib_PA_Line : test == "pa_pattern" ? CalibMode::Calib_PA_Pattern : CalibMode::Calib_PA_Tower;
        p.calib_pa(cp);
    } else if (test == "flow_rate") {
        p.calib_flowrate(arg<bool>(params, "linear", false), arg<int>(params, "pass", 1));
    } else if (test == "temp_tower") {
        cp.mode = CalibMode::Calib_Temp_Tower;
        p.calib_temp(cp);
    } else if (test == "max_volumetric_speed") {
        cp.mode = CalibMode::Calib_Vol_speed_Tower;
        p.calib_max_vol_speed(cp);
    } else if (test == "vfa") {
        cp.mode = CalibMode::Calib_VFA_Tower;
        p.calib_VFA(cp);
    } else if (test == "retraction") {
        cp.mode = CalibMode::Calib_Retraction_tower;
        p.calib_retraction(cp);
    } else if (test == "input_shaping_freq") {
        cp.mode = CalibMode::Calib_Input_shaping_freq;
        p.calib_input_shaping_freq(cp);
    } else if (test == "input_shaping_damp") {
        cp.mode = CalibMode::Calib_Input_shaping_damp;
        p.calib_input_shaping_damp(cp);
    } else if (test == "cornering") {
        cp.mode = CalibMode::Calib_Cornering;
        p.Calib_Cornering(cp);
    } else
        throw ToolError("unknown test '" + test + "'", -32602);
    return json{{"test", test}, {"objects", p.model().objects.size()}};
}

json arrange(const json&)
{
    if (!plater().can_arrange())
        throw ToolError("nothing to arrange");
    plater().arrange();
    return json{{"started", true}};
}

json orient(const json& params)
{
    if (params.contains("objects"))
        select_objects(object_indices(params));
    plater().orient();
    return json{{"started", true}};
}

json objects_after_job(const json&) { return objects_list(json()); }

} // namespace

void register_slicer_tools()
{
    const json obj = param("object", "integer", "object index (objects_list)");
    const json plate_opt = param("plate", "integer", "plate index; omit for the current plate", -1);

    // project
    register_tool({"project_new", "Start a new empty project (File > New).",
                   json::array({param("discard_changes", "boolean", "required when the project has unsaved changes", false)}), 0, project_new});
    register_tool({"project_open", "Open a .3mf project (File > Open). Preset-change prompts may appear: answer them with ui_* tools.",
                   json::array({param("path", "string", "absolute path"), param("discard_changes", "boolean", "", false)}), Long | Waitable, project_open,
                   [](const json&) { return objects_list(json()); }});
    register_tool({"project_save", "Save the project, to `path` (Save As) or to its current file.",
                   json::array({param("path", "string", "absolute .3mf path; omit to save in place", "")}), Long, project_save});
    register_tool({"project_import", "Import models (STL, 3MF, STEP, OBJ, AMF, SVG...) into the project, as File > Import does.",
                   json::array({param("paths", "array", "absolute file paths"),
                                param("load_settings", "boolean", "also load settings embedded in 3MF files", false)}),
                   Long, project_import});
    register_tool({"project_export_3mf", "Export the project (or one plate) as a 3MF without changing the current project file.",
                   json::array({param("path", "string", "absolute .3mf path"), param("plate", "integer", "only this plate; -1 = all", -1)}),
                   Long, project_export_3mf});
    register_tool({"model_export", "Export objects as one STL or OBJ mesh in world coordinates (all instances, negative parts subtracted).",
                   json::array({param("path", "string", "absolute .stl/.obj path"), param("objects", "array", "object indices; omit for all", json::array()),
                                param_enum("format", json::array({"stl", "obj"}), "default from the extension", "stl")}),
                   Long, model_export});
    register_tool({"gcode_preview_file", "Open a G-code file in the preview (File > Import G-code).",
                   json::array({param("path", "string", "absolute .gcode path")}), Long, load_gcode});

    // objects
    register_tool({"objects_list", "Objects in the project: index, name, parts, instances, plate, bounding box, override counts.",
                   json::array({param("detail", "boolean", "include parts, instances, settings and layer ranges of every object", false)}),
                   0, objects_list});
    register_tool({"object_get", "One object in detail: parts (type, filament, mesh), instances (position, rotation, scale), settings overrides, layer ranges.",
                   json::array({obj}), 0, object_get});
    register_tool({"objects_select", "Select objects (or one part) in the 3D view and object list; empty = select none.",
                   json::array({param("objects", "array", "object indices; empty or omitted = deselect", json::array()),
                                param("instance", "integer", "only this instance", -1), param("volume", "integer", "select this part of the single object", -1),
                                param("all", "boolean", "select everything", false)}),
                   0, objects_select});
    register_tool({"objects_delete", "Delete objects.",
                   json::array({param("objects", "array", "object indices", json::array()), param("all", "boolean", "delete every object", false)}),
                   0, objects_delete});
    register_tool({"object_transform", "Move / rotate / scale / resize one instance of an object. Rotation is XYZ Euler degrees.",
                   json::array({obj, param("instance", "integer", "", 0), param("position", "array", "[x,y,z] mm (world)", json()),
                                param("rotation", "array", "[rx,ry,rz] degrees", json()), param("scale", "array", "[sx,sy,sz] factors", json()),
                                param("size", "array", "[x,y,z] target bounding-box size mm; 0 keeps an axis", json()),
                                param("uniform", "boolean", "with size: scale all axes by the first given axis's factor", false),
                                param("relative", "boolean", "add to the current position/rotation, multiply the scale", false),
                                param("drop_to_bed", "boolean", "put the object back on the bed", true)}),
                   0, object_transform});
    register_tool({"object_set", "Rename an object, set it printable or not, or assign its filament.",
                   json::array({obj, param("name", "string", "", json()), param("printable", "boolean", "", json()),
                                param("filament", "integer", "1-based filament slot; 0 = default", json())}),
                   0, object_set});
    register_tool({"object_instances", "Set how many instances (copies) of an object there are.",
                   json::array({obj, param("count", "integer", "1..1000")}), 0, object_instances});
    register_tool({"object_mirror", "Mirror an object (or one instance) along an axis.",
                   json::array({obj, required(param_enum("axis", json::array({"x", "y", "z"}), "", json())), param("instance", "integer", "", -1)}),
                   0, object_mirror});
    register_tool({"object_split", "Split an object into separate objects or into parts, by connected mesh shells.",
                   json::array({obj, param_enum("mode", json::array({"objects", "parts"}), "", "objects")}), 0, object_split});
    register_tool({"object_cut", "Cut an object with a horizontal plane at height z above the bed.",
                   json::array({obj, param("z", "number", "cut height, mm"), param("instance", "integer", "", 0),
                                param_enum("keep", json::array({"both", "upper", "lower"}), "", "both"),
                                param("keep_as_parts", "boolean", "keep both halves as parts of one object", false),
                                param("flip_upper", "boolean", "flip the upper part upside down", false),
                                param("place_on_cut", "boolean", "place the parts on their cut faces", false)}),
                   0, object_cut});
    register_tool({"object_add_shape", "Add a primitive as a new object, or as a part / modifier / negative volume / support blocker / enforcer of an object.",
                   json::array({required(param_enum("shape", json::array({"Cube", "Cylinder", "Sphere", "Cone", "Disc", "Torus"}), "", json())),
                                param_enum("as", json::array({"object", "part", "negative", "modifier", "support_blocker", "support_enforcer"}), "", "object"),
                                param("object", "integer", "target object unless as=object", json())}),
                   0, object_add_shape});
    register_tool({"volume_set", "Change a part of an object: its type, name or filament.",
                   json::array({obj, param("volume", "integer", "part index (object_get)"), param_enum("type", k_volume_types, "", json()),
                                param("name", "string", "", json()), param("filament", "integer", "1-based slot; 0 = the object's", json())}),
                   0, volume_set});
    register_tool({"volume_delete", "Delete a part of an object.", json::array({obj, param("volume", "integer", "part index")}), 0, volume_delete});
    register_tool({"object_settings_get", "Settings overridden on an object, one of its parts, or one of its layer ranges.",
                   json::array({obj, param("volume", "integer", "a part instead of the object", json()),
                                param("layer_range", "array", "[min_z, max_z] instead of the object", json())}),
                   0, object_settings_get});
    register_tool({"object_settings_set",
                   "Override print settings for an object, a part (modifier) or a height range; `reset` removes overrides. A new layer_range is created on first use.",
                   json::array({obj, param("settings", "object", "{key: value}, values as config_get prints them", json::object()),
                                param("reset", "array", "keys whose override to remove", json::array()),
                                param("volume", "integer", "a part instead of the object", json()),
                                param("layer_range", "array", "[min_z, max_z] mm", json()),
                                param("remove_range", "boolean", "with layer_range: delete that range", false)}),
                   0, object_settings_set});
    register_tool({"arrange", "Arrange all objects on the plates (as the Arrange button).", json::array(), Waitable, arrange, objects_after_job});
    register_tool({"orient", "Auto-orient objects for printing (selected ones, or all).",
                   json::array({param("objects", "array", "object indices; omit for the current selection / all", json::array())}),
                   Waitable, orient, objects_after_job});

    // plates
    register_tool({"plates_list", "Plates: name, objects on each, bed type, print sequence, lock, slice state and errors.",
                   json::array(), 0, plates_list});
    register_tool({"plate_select", "Make a plate the current one.", json::array({param("plate", "integer", "")}), 0, plate_select});
    register_tool({"plate_add", "Add an empty plate.", json::array(), 0, plate_add});
    register_tool({"plate_delete", "Delete a plate and the objects on it.", json::array({param("plate", "integer", "")}), 0, plate_delete});
    register_tool({"plate_duplicate", "Duplicate a plate with its objects.", json::array({param("plate", "integer", "")}), 0, plate_duplicate});
    register_tool({"plate_set", "Change a plate's name, lock, bed type, print sequence or spiral vase mode.",
                   json::array({param("plate", "integer", ""), param("name", "string", "", json()), param("locked", "boolean", "", json()),
                                param("bed_type", "string", "'default' or a curr_bed_type value (config_describe key=curr_bed_type)", json()),
                                param("print_sequence", "string", "default | by layer | by object", json()),
                                param("spiral_vase", "boolean", "", json())}),
                   0, plate_set});
    register_tool({"object_move_to_plate", "Move an object instance to another plate, keeping its place on the plate.",
                   json::array({obj, param("plate", "integer", "target plate"), param("instance", "integer", "", 0)}), 0, object_move_to_plate});

    // presets
    register_tool({"presets_list", "Presets of a type, the selected one and whether it has unsaved changes; for filaments also the slot assignment.",
                   json::array({param_preset_type(), param("include_incompatible", "boolean", "", false), param("include_hidden", "boolean", "", false)}),
                   0, presets_list});
    register_tool({"preset_select", "Select a printer, print or filament preset (filaments per slot), as the sidebar combos do.",
                   json::array({param_preset_type(), param("name", "string", "preset name (presets_list)"),
                                param("slot", "integer", "filament slot, 0-based", 0),
                                param("discard_changes", "boolean", "drop unsaved changes of the currently selected preset", false)}),
                   0, preset_select});
    register_tool({"preset_save", "Save the edited preset's changes, as a new user preset when `name` is given.",
                   json::array({param_preset_type(), param("name", "string", "omit to overwrite the selected user preset", ""),
                                param("detach", "boolean", "save as a preset not inheriting from a system one", false)}),
                   0, preset_save});
    register_tool({"preset_discard", "Discard unsaved changes of the selected preset.", json::array({param_preset_type()}), 0, preset_discard});
    register_tool({"preset_diff", "Unsaved changes of the selected preset: key -> {edited, saved}.", json::array({param_preset_type()}), 0, preset_diff});
    register_tool({"filament_add", "Add a filament slot, optionally with a preset.",
                   json::array({param("preset", "string", "filament preset for the new slot", "")}), 0, filament_add});
    register_tool({"filament_remove", "Remove a filament slot.",
                   json::array({param("slot", "integer", "0-based"), param("replace_with", "integer", "slot whose filament objects using the removed one get; -1 = default", -1)}),
                   0, filament_remove});

    // settings
    register_tool({"config_describe", "Describe settings: label, category, tooltip, type, unit, limits, enum values, default, current value, and which preset owns it. Search by key, text or category.",
                   json::array({param("key", "string", "one exact key", ""), param("search", "string", "substring of key / label / tooltip", ""),
                                param("category", "string", "e.g. Quality, Strength, Speed, Support, Others", ""), param("limit", "integer", "", 50)}),
                   0, config_describe});
    register_tool({"config_get", "Current values of settings (keys), or every setting of the edited print / filament / printer preset (type).",
                   json::array({param("keys", "array", "setting keys", json()), param_enum("type", json::array({"print", "filament", "printer"}), "", json())}),
                   0, config_get});
    register_tool({"config_set",
                   "Change any print, filament, printer or project setting, as typing it in the settings tab does: the preset becomes modified (preset_save / preset_discard). "
                   "Values use the config's own serialisation; numbers and booleans are accepted.",
                   json::array({param("settings", "object", "{key: value}"),
                                param_enum("type", json::array({"print", "filament", "printer"}), "force the preset the keys belong to", json())}),
                   0, config_set});

    // slicing / G-code
    register_tool({"slice", "Slice the current plate, one plate, or all plates (as the Slice buttons).",
                   json::array({param("plate", "string", "\"current\", \"all\" or a plate index", "current"),
                                param("show_preview", "boolean", "switch to the Preview tab, as the button does", true)}),
                   Waitable, slice, slicing_status});
    register_tool({"slicing_status", "Per-plate slice state (sliced, progress, error, G-code warnings) plus validation of the current plate.",
                   json::array({param("plate", "integer", "only this plate", -1)}), ModalSafe, slicing_status});
    register_tool({"slice_cancel", "Cancel the running slicing.", json::array(), 0, slice_cancel});
    register_tool({"gcode_stats", "Statistics of a sliced plate: time, filament length/weight/cost, layers, changes, warnings.",
                   json::array({plate_opt}), 0, gcode_stats});
    register_tool({"gcode_export", "Export a plate's G-code to a file (slicing it first if needed), post-processing included.",
                   json::array({param("path", "string", "absolute .gcode path"), param("plate", "integer", "default current", json())}),
                   Waitable | Long, gcode_export, gcode_export_result});
    register_tool({"gcode_read", "Read lines of a sliced plate's G-code (or any G-code file): a window, or the lines containing `grep`.",
                   json::array({plate_opt, param("path", "string", "read this file instead", ""), param("offset", "integer", "skip this many lines", 0),
                                param("max_lines", "integer", "", 200), param("grep", "string", "only lines containing this text", "")}),
                   0, gcode_read});

    // printers / calibration
    register_tool({"printers_list", "Configured network printers (OctoPrint, Moonraker, PrusaLink...) and the printer preset's host.",
                   json::array(), 0, printers_list});
    register_tool({"printer_upload", "Upload a G-code file to a network printer, optionally starting the print.",
                   json::array({param("path", "string", "G-code file (gcode_export first)"),
                                param("printer", "string", "physical printer name; default the selected one, else the printer preset's host", ""),
                                param("remote_name", "string", "file name on the printer", ""), param("group", "string", "", ""),
                                param("start_print", "boolean", "start printing after the upload", false)}),
                   0, printer_upload});
    register_tool({"calibrate", "Create a calibration project (replaces the current project; may ask to save it).",
                   json::array({required(param_enum("test", json::array({"pa_line", "pa_pattern", "pa_tower", "flow_rate", "temp_tower", "max_volumetric_speed", "vfa", "retraction", "input_shaping_freq", "input_shaping_damp", "cornering"}), "", json())),
                                param("start", "number", "", 0), param("end", "number", "", 1), param("step", "number", "", 0.1),
                                param("extruder", "integer", "", 0), param("print_numbers", "boolean", "", false),
                                param("linear", "boolean", "flow_rate: linear test", false), param("pass", "integer", "flow_rate: 1 or 2", 1),
                                param("freq_start_x", "number", "", 0), param("freq_end_x", "number", "", 1),
                                param("freq_start_y", "number", "", 0), param("freq_end_y", "number", "", 1),
                                param("shaper_type", "string", "", ""), param("test_model", "integer", "", 0),
                                param("nozzle_based_resize", "boolean", "", true),
                                param("accelerations", "array", "", json::array()), param("speeds", "array", "", json::array())}),
                   Long, calibrate});
}

}}} // namespace Slic3r::GUI::Mcp
