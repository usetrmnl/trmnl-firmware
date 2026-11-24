# USB Host CDC-ECM Class Driver

This component contains an implementation of a USB CDC-ECM Host Class Driver that is implemented on top of the [USB Host Library](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/usb_host.html).

It is based on the USB CDC-ACM Host Class Driver provided by Espressif at https://github.com/espressif/esp-usb/blob/b79a9c25ce77d89e934023205ef184e3be1bd59b/host/class/cdc/usb_host_cdc_acm/

## Supported Devices

The CDC-ECM Host driver supports the following types of CDC devices:

1. CDC-ECM devices (e.g. RealTek 8152 / 8153 usb-to-ethernet devices)



## Usage

The following steps outline the typical API call pattern of the CDC-ACM Class Driver

1. Specify CDC-ECM parameters via `static ece_ecm_params_t cdc_ecm_params`
2. Initialise the CDC-ECM driver by calling `cdc_ecm_init(&cdc_ecm_params)`

```
e.g.:
static cdc_ecm_params_t cdc_ecm_params = {
    .vid = 0x0BDA,
    .pids = {0x8152, 0x8153},
    .event_cb = netif_event_handler,
    .callback_arg = NULL,
    .if_key = "cdc_ecm_host",
    .if_desc = "usb cdc ecm host device",
    .hostname = "Espressif CDC-ECM Host Device"
}
```


## Example

- For an example with a CDC-ECM device Realtek 8152, refer to [cdc-ecm-host-exmaple](https://github.com/gadget-man/cdc-ecm-host-example)
