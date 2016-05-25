#ifndef NODE_NAMES_H
#define NODE_NAMES_H

namespace skiros_config
{
    const char skiros_namespace[] = "skiros";
    //Params
    const char scene_workspace[] = "scene_workspace";
    const char owl_workspace[] = "owl_workspace";
    const char save_log[] = "save_log";
    //Node names
    const char world_model_node_name[] = "skiros_wm";
    const char skill_mgr_node_name[] = "aau_stamina_robot";
    const char task_mgr_node_name[] = "skiros_task_manager";

    //Task manager topics\services
    const char task_query_srv_name[] = "/task_query";
    const char task_modify_srv_name[] = "/task_modify";
    const char task_exe_tpc_name[] = "/task_exe";
    const char task_monitor_tpc_name[] = "/monitor";
    const char task_plan_srv_name[] = "/task_plan";

    //Skill manager topics\services
    const char module_list_query_srv_name[] = "/module_list_query";
    const char skill_list_query_srv_name[] = "/skill_list_query";
    const char skill_cmd_srv_name[] = "/skill_command";
    const char module_cmd_srv_name[] = "/module_command";
    const char skill_exe_srv_name[] = "/skill_exe";
    const char module_exe_srv_name[] = "/module_exe";
    const char skill_monitor_tpc_name[] = "/monitor";

    //World model topics\services
    const char wm_monitor_tpc_name[] = "/monitor";
    const char wm_query_ont_srv_name[] = "/query_ontology";
    const char wm_modify_ont_srv_name[] = "/modify_ontology";
    const char wm_query_mdl_srv_name[] = "/query_model";
    const char wm_set_rel_srv_name[] = "/set_relation";
    const char wm_get_elm_srv_name[] = "/element_get";
    const char wm_mdf_elm_srv_name[] = "/element_modify";
    const char wm_classify_srv_name[] = "/classify";
    const char wm_identify_srv_name[] = "/identify";
    const char wm_scene_load_save_srv_name[] = "/scene_load_and_save";
    const char wm_lock_unlock_srv_name[] = "/lock_unlock";
}

#endif //NODE_NAMES_H
