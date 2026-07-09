/*
 * HCICS local deployment configuration example.
 *
 * Copy this file to applications/hcics_config.h for a local deployment and
 * define HCICS_USE_LOCAL_CONFIG in the build if you want hcics_app.c to include
 * the private file. Do not commit the private copy.
 */

#ifndef HCICS_CONFIG_EXAMPLE_H
#define HCICS_CONFIG_EXAMPLE_H

#define HCICS_WIFI_SSID      "HCICS-ESP32S3"
#define HCICS_WIFI_PASSWORD  "replace-with-ap-password"
#define HCICS_UDP_TOKEN      "replace-with-shared-token"

#endif /* HCICS_CONFIG_EXAMPLE_H */
