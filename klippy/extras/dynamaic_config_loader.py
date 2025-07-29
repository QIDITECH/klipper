# Dynamic configuration loader for Klipper
#
# Copyright (C) 2023  Your Name <your.email@example.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import configparser
import importlib
import os

class DynamicConfigLoader:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config_path = config.get('config_path')
        self.loaded_configs = {}
        
        # 注册命令
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('LOAD_CONFIG', self.cmd_LOAD_CONFIG,
                               desc="动态加载配置文件")
        gcode.register_command('UNLOAD_CONFIG', self.cmd_UNLOAD_CONFIG,
                               desc="卸载动态加载的配置")
        gcode.register_command('LIST_LOADED_CONFIGS', self.cmd_LIST_LOADED_CONFIGS,
                               desc="列出所有已加载的动态配置")
        
        # 在printer对象初始化完成后加载默认配置
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
    
    def handle_connect(self):
        if self.config_path and os.path.exists(self.config_path):
            self._load_config_file(self.config_path)
    
    def _load_config_file(self, config_path):
        try:
            config_parser = configparser.ConfigParser()
            config_parser.read(config_path)
            
            for section in config_parser.sections():
                module_name = section.split()[0]
                
                # 尝试导入模块
                try:
                    module = importlib.import_module('extras.' + module_name)
                    if hasattr(module, 'load_config'):
                        # 创建配置对象
                        config_obj = self.printer.lookup_object('configfile').create_config_wrapper(
                            config_parser, section)
                        # 调用模块的load_config函数
                        obj = module.load_config(config_obj)
                        if obj is not None:
                            # 将对象添加到printer
                            self.printer.add_object(section, obj)
                            self.loaded_configs[section] = {
                                'module': module_name,
                                'object': obj
                            }
                            logging.info("动态加载配置: %s", section)
                except ImportError:
                    logging.warning("无法导入模块 %s", module_name)
                except Exception as e:
                    logging.exception("加载配置 %s 时出错: %s", section, str(e))
            
            return True
        except Exception as e:
            logging.exception("加载配置文件 %s 时出错: %s", config_path, str(e))
            return False
    
    def cmd_LOAD_CONFIG(self, gcmd):
        config_path = gcmd.get('PATH')
        if not os.path.exists(config_path):
            raise gcmd.error("配置文件不存在: %s" % config_path)
        
        success = self._load_config_file(config_path)
        if success:
            gcmd.respond_info("成功加载配置文件: %s" % config_path)
        else:
            raise gcmd.error("加载配置文件失败: %s" % config_path)
    
    def cmd_UNLOAD_CONFIG(self, gcmd):
        section = gcmd.get('SECTION')
        if section not in self.loaded_configs:
            raise gcmd.error("未找到已加载的配置: %s" % section)
        
        # 从printer中移除对象
        try:
            # 某些对象可能需要特殊的清理逻辑
            obj = self.loaded_configs[section]['object']
            if hasattr(obj, 'shutdown'):
                obj.shutdown()
            
            # 从loaded_configs中移除
            del self.loaded_configs[section]
            gcmd.respond_info("成功卸载配置: %s" % section)
        except Exception as e:
            logging.exception("卸载配置 %s 时出错: %s", section, str(e))
            raise gcmd.error("卸载配置失败: %s" % section)
    
    def cmd_LIST_LOADED_CONFIGS(self, gcmd):
        if not self.loaded_configs:
            gcmd.respond_info("没有动态加载的配置")
            return
        
        msg = "已加载的动态配置:\n"
        for section, info in self.loaded_configs.items():
            msg += "  %s (模块: %s)\n" % (section, info['module'])
        gcmd.respond_info(msg)

def load_config(config):
    return DynamicConfigLoader(config)