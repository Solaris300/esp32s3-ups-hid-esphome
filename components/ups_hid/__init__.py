import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

ups_hid_ns = cg.esphome_ns.namespace('ups_hid')
UpsHid = ups_hid_ns.class_('UpsHid', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(UpsHid),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
