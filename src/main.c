/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos.h"
#include "mgos_gpio.h"
#include "mgos_sys_config.h"
#include "frozen.h"
#include "mgos_time.h"
#include <time.h>
//#include "mgos_sntp.h"
#include "sys/stat.h"
#include "mgos_i2c.h"
#include "mgos_ds3231.h"
#include <stdio.h>
#include <stdint.h>
#include "mgos_mqtt.h"
#include <stdlib.h>
//#include "mgos_mdash_api.h"


static const char *topic_str = NULL;
static const char *filename = "encoder.json";
static char *json_str_msg = NULL;
static char *last_json_str_msg = NULL;
static char *last_json_str_msg2 = NULL;
static int64_t position = 0;
static int64_t last_position = 0;
static int64_t start_position = 0;
static int64_t end_position = 0;
static int64_t start_service_position = 0;
static int64_t end_service_position = 0;
static int64_t current = 0;
static int64_t folio = 1;
static int64_t reporte = 1;
static int64_t start_delta_time = 0;
static int64_t end_delta_time = 0;
static int64_t dif_delta_time = 0;
static int64_t last_dif_delta_time = 0;
static int lastEncoded = 0;
static int delta = 2;
static int delta_pulses = 0;
static int on_service = 0;
static int stop_time = 10;   // Segundos después para declarar el fin del servicio
static int delta_time = 5;
static int delta_counter = 5;
static int valve_state;
static int timer_counter;
static time_t start_time, end_time, actual_time;
static int litros = 0;
static int precio = 0;
static float pulsos_litro = 1;
static float precio_litro =  9.8;
char time_str[9];
char date_str[12];
char folio_str[4];


int ENCODER_PIN_A;
int ENCODER_PIN_B;
int LED_PIN;
int VALVE_PIN;
int UART_NO = 0;
int UART_NO1 = 2;
#define MGOS_DS3231_DEFAULT_I2C_ADDR  104
//int addr = 104;

// Declaración del manejador del DS3231
struct mgos_ds3231 *rtc = NULL;
time_t now;
struct tm *t;

//static const char *log_folder = "folios";
//static const char *report_folder = "reportes";
// Comandos de inicialización y formateo de texto ESC/POS
  uint8_t init_cmd[] = {0x1B, 0x40};  // Inicializar la impresora
  uint8_t center_cmd[] = {0x1B, 0x61, 0x01};  // Centrar texto
  uint8_t left_cmd[] = {0x1B, 0x61, 0x00};  // Alinear a la izquierda
  uint8_t double_height_cmd[] = {0x1B, 0x21, 0x30};  // Doble altura
  uint8_t normal_size_cmd[] = {0x1B, 0x21, 0x00};  // Tamaño normal
  uint8_t cut_paper_cmd[] = {0x1B, 0x6D};  // Cortar papel
  uint8_t end_print_cmd[] = {0x1D, 0x56, 0x42};  // Cortar papel
  //ser.write(b'\x1D\x56\x42\x00')
  //ser.write(b'\x1B\x6D\x42\x00')



// ------------------------------------------------------------- send_to_display
void send_to_display() {
  json_str_msg = json_asprintf("{method: med, params: {Litros: \"%d\", Precio: \"%d\"}}", litros, precio);

  if (json_str_msg != NULL) {
    if (last_json_str_msg == NULL || strcmp(last_json_str_msg, json_str_msg) != 0) {
      mgos_uart_printf(UART_NO, "%s\n", json_str_msg);

      // Liberar la memoria del último mensaje
      if (last_json_str_msg != NULL) {
        free(last_json_str_msg);
      }

      // Guardar el nuevo mensaje como el último mensaje enviado
      last_json_str_msg = json_str_msg;
    } else {
      // Si el mensaje es el mismo, liberar la memoria del mensaje actual
      free(json_str_msg);
    }
  }

  now = time(NULL);
  t = localtime(&now);

  snprintf(time_str, sizeof(time_str), "%02d:%02d", t->tm_hour,t->tm_min); 
  strftime(date_str, sizeof(date_str), "%d/%b/%Y", t);
  snprintf(folio_str, sizeof(folio_str), "%lld", folio);

  char *json_str_msg2 = json_asprintf("{method: time, params: {hour: \"%s\", day:\"%s\", folio:\"%s\"}}", time_str, date_str, folio_str);
  if (json_str_msg2 != NULL) {
    if (last_json_str_msg2 == NULL || strcmp(last_json_str_msg2, json_str_msg2) != 0) {
      mgos_uart_printf(UART_NO, "%s\n", json_str_msg2);

      // Liberar la memoria del último mensaje
      if (last_json_str_msg2 != NULL) {
        free(last_json_str_msg2);
      }

      // Guardar el nuevo mensaje como el último mensaje enviado
      last_json_str_msg2 = json_str_msg2;
    } else {
      // Si el mensaje es el mismo, liberar la memoria del mensaje actual
      free(json_str_msg2);
    }
  }

}


//----------------------------------------------------- print_report
void print_report_warning(char *json_str)
{
  mgos_uart_write(UART_NO1, init_cmd, sizeof(init_cmd));
  mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
  mgos_uart_write(UART_NO1, normal_size_cmd, sizeof(normal_size_cmd));
  mgos_uart_printf(UART_NO1, json_str);
  mgos_uart_write(UART_NO1, "\n", 1);
  mgos_uart_write(UART_NO1, end_print_cmd, sizeof(end_print_cmd));
  mgos_uart_write(UART_NO1, cut_paper_cmd, sizeof(cut_paper_cmd));
}

//----------------------------------------------------- print_report
void print_report(char *json_str)
{
  
  mgos_uart_write(UART_NO1, init_cmd, sizeof(init_cmd));
  mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
  
  // Imprimir fecha y folio desde el JSON
  struct json_token total_folios_tok, first_folio_tok, last_folio_tok,total_litros_tok,total_precio_tok,reporte_tok, actual_time_tok;
  if (json_scanf(json_str, strlen(json_str), "{total_folios: %T, first_folio: %T, last_folio: %T, total_litros: %T, total_precio: %T, reporte: %T, actual_time: %T}", 
                &total_folios_tok, &first_folio_tok, &last_folio_tok, &total_litros_tok, &total_precio_tok,&reporte_tok, &actual_time_tok) == 7) {
    char total_folios[32], first_folio[32], last_folio[32], total_litros[32], total_precio[32], reporte[32], actual_time[32];
    snprintf(total_folios, total_folios_tok.len + 1, "%.*s", total_folios_tok.len, total_folios_tok.ptr);
    snprintf(first_folio, first_folio_tok.len + 1, "%.*s", first_folio_tok.len, first_folio_tok.ptr);
    snprintf(last_folio, last_folio_tok.len + 1, "%.*s", last_folio_tok.len, last_folio_tok.ptr);
    snprintf(total_litros, total_litros_tok.len + 1, "%.*s", total_litros_tok.len, total_litros_tok.ptr);
    snprintf(total_precio, total_precio_tok.len + 1, "%.*s", total_precio_tok.len, total_precio_tok.ptr);
    snprintf(reporte, reporte_tok.len + 1, "%.*s", reporte_tok.len, reporte_tok.ptr);
    snprintf(actual_time, actual_time_tok.len + 1, "%.*s", actual_time_tok.len, actual_time_tok.ptr);


    const char *id =  mgos_sys_config_get_app_id();
    const char *header =  mgos_sys_config_get_app_tiket_header();

    mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
  
    mgos_uart_write(UART_NO1, double_height_cmd, sizeof(double_height_cmd));
    mgos_uart_printf(UART_NO1, "REPORTE: %s\n", reporte);
    mgos_uart_write(UART_NO1, normal_size_cmd, sizeof(normal_size_cmd));
    //mgos_uart_write(UART_NO1, "\n", 1);

    mgos_uart_printf(UART_NO1, "Equipo: %s\n", id);
    mgos_uart_write(UART_NO1, left_cmd, sizeof(left_cmd));
    mgos_uart_printf(UART_NO1, "Fecha: %s\n", actual_time);

    mgos_uart_printf(UART_NO1, "Inicio: %s\t", first_folio);
    mgos_uart_printf(UART_NO1, "Fin: %s\n", last_folio);

    // Imprimir litros en doble altura
    mgos_uart_write(UART_NO1, double_height_cmd, sizeof(double_height_cmd));
    mgos_uart_printf(UART_NO1, "LITROS: %s L\n", total_litros);
    
    // Imprimir precio en doble altura
    mgos_uart_printf(UART_NO1, "VENTA: $%s\n", total_precio);

    // Restaurar tamaño de texto normal
    mgos_uart_write(UART_NO1, normal_size_cmd, sizeof(normal_size_cmd));
    mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
    mgos_uart_write(UART_NO1, "\n", 1);
  
    //const char *footer =  mgos_sys_config_get_app_tiket_footer();
    //mgos_uart_write(UART_NO1, footer, strlen(footer));
  
  } else {
    LOG(LL_ERROR, ("Error parsing JSON"));
  }

  
  mgos_uart_write(UART_NO1, end_print_cmd, sizeof(end_print_cmd));
  mgos_uart_write(UART_NO1, cut_paper_cmd, sizeof(cut_paper_cmd));
}

//----------------------------------------------------- print_tiket
void print_tiket(char *json_str) {
  

  mgos_uart_write(UART_NO1, init_cmd, sizeof(init_cmd));
  mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
  
  const char *header =  mgos_sys_config_get_app_tiket_header();
  mgos_uart_write(UART_NO1, header, strlen(header));
  mgos_uart_write(UART_NO1, "\n", 1);
  
  mgos_uart_write(UART_NO1, left_cmd, sizeof(left_cmd));
  
  // Imprimir fecha y folio desde el JSON
  struct json_token date_tok, folio_tok, litros_tok, precio_tok, precio_litro_tok;
  if (json_scanf(json_str, strlen(json_str), "{e_time: %T, folio: %T, litros: %T, precio: %T, precio_l: %T}", 
                &date_tok, &folio_tok, &litros_tok, &precio_tok, &precio_litro_tok) == 5) {
    char date[32], folio[32], litros[32], precio[32], precio_litro[32];
    snprintf(date, date_tok.len + 1, "%.*s", date_tok.len, date_tok.ptr);
    snprintf(folio, folio_tok.len + 1, "%.*s", folio_tok.len, folio_tok.ptr);
    snprintf(litros, litros_tok.len + 1, "%.*s", litros_tok.len, litros_tok.ptr);
    snprintf(precio, precio_tok.len + 1, "%.*s", precio_tok.len, precio_tok.ptr);
    snprintf(precio_litro, precio_litro_tok.len + 1, "%.*s", precio_litro_tok.len, precio_litro_tok.ptr);


    const char *id =  mgos_sys_config_get_app_id();

    mgos_uart_printf(UART_NO1, "Fecha: %s\n", date);
    mgos_uart_printf(UART_NO1, "Folio: %s\t", folio);
    mgos_uart_printf(UART_NO1, "Equipo: %s\n", id);
    mgos_uart_printf(UART_NO1, "$ Unitario: %s\n", precio_litro);

    // Imprimir litros en doble altura
    mgos_uart_write(UART_NO1, double_height_cmd, sizeof(double_height_cmd));
    mgos_uart_printf(UART_NO1, "LITROS: %s L\n", litros);
    
    // Imprimir precio en doble altura
    mgos_uart_printf(UART_NO1, "VENTA: $%s\n", precio);

    // Restaurar tamaño de texto normal
    mgos_uart_write(UART_NO1, normal_size_cmd, sizeof(normal_size_cmd));
    mgos_uart_write(UART_NO1, center_cmd, sizeof(center_cmd));
    mgos_uart_write(UART_NO1, "\n", 1);
  
    const char *footer =  mgos_sys_config_get_app_tiket_footer();
    mgos_uart_write(UART_NO1, footer, strlen(footer));
  
  } else {
    LOG(LL_ERROR, ("Error parsing JSON"));
  }

  
  mgos_uart_write(UART_NO1, end_print_cmd, sizeof(end_print_cmd));
  mgos_uart_write(UART_NO1, cut_paper_cmd, sizeof(cut_paper_cmd));
}


// --------------------------------------------------- mqtt_handler
static void handler(struct mg_connection *c, const char *topic, int topic_len,
                    const char *msg, int msg_len, void *userdata) {
  LOG(LL_INFO, ("Got message on topic %.*s", topic_len, topic));
}

// Crear carpeta si no existe ----------------------create_folder
void create_folder(const char *folder_path) {
  struct stat st = {0};
  if (stat(folder_path, &st) == -1) {
    if (mkdir(folder_path, 0700) != 0) {
      LOG(LL_ERROR, ("Failed to create directory: %s", folder_path));
    }
    else
    {
      LOG(LL_INFO, ("Create Dir"));
    }

  }
  else
  {
     LOG(LL_INFO, ("Create Dir???"));
  }
}


//---------------------------------------------- make_report
void make_report() {
  //create_folder(report_folder);

  char log_file_path[100], report_file_path[100];
  actual_time = time(NULL);
  struct tm *t = localtime(&actual_time);
  //snprintf(log_file_path, sizeof(log_file_path), "fol_%04d_%02d_%02d.json", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
  snprintf(log_file_path, sizeof(log_file_path), "rep_%lld.json", reporte);
  //snprintf(report_file_path, sizeof(report_file_path), "rep_%lld_%04d_%02d_%02d.json", reporte, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
  snprintf(report_file_path, sizeof(report_file_path), "rep_%lld.json", reporte);

  char actual_time_str[20];
  strftime(actual_time_str, sizeof(actual_time_str), "%Y-%m-%d %H:%M:%S", localtime(&actual_time));

  FILE *log_file = fopen(log_file_path, "r");
  if (log_file == NULL) {
    LOG(LL_ERROR, ("Failed to open log file: %s", log_file_path));
    print_report_warning("No hay reporte nuevo");
    return;
  }

  int64_t folio_sum = 0;
  int64_t first_folio = -1;
  int64_t last_folio = -1;
  int64_t total_litros = 0;
  int64_t total_precio = 0;

  char line[512];
  while (fgets(line, sizeof(line), log_file) != NULL) {
    int64_t folio, litros, precio;
    if (json_scanf(line, strlen(line), "{folio: %lld, litros: %lld, precio: %lld}", &folio, &litros, &precio) == 3) {
      folio_sum++;
      if (first_folio == -1) {
        first_folio = folio;
      }
      last_folio = folio;
      total_litros += litros;
      total_precio += precio;
    }
  }
  fclose(log_file);

  char *report_str = json_asprintf(
    "{"
    "total_folios: %lld,"
    "first_folio: %lld,"
    "last_folio: %lld,"
    "total_litros: %lld,"
    "total_precio: %lld,"
    "reporte: %lld,"
    "actual_time: \"%s\""
    "}\n",
    folio_sum, first_folio, last_folio, total_litros, total_precio, reporte, actual_time_str
  );

  LOG(LL_INFO, ("Reporte: %s", report_str));

  if (report_str != NULL) {
    FILE *report_file = fopen(report_file_path, "a");
    if (report_file != NULL) {
      fwrite(report_str, 1, strlen(report_str), report_file);
      fclose(report_file);
      LOG(LL_INFO, ("Report saved to file: %s", report_file_path));
      if(total_litros > 0)
      {
        print_report(report_str);
        reporte++;
        //mgos_mqtt_pub("my/topic", report_str, strlen(report_str), 1, 0); 
      char full_topic_str[128];
      snprintf(full_topic_str, sizeof(full_topic_str), "%s/out", topic_str);
      mgos_mqtt_pub(full_topic_str, report_str, strlen(report_str), 1, 0);  /* Publish */
      }
    } else {
      LOG(LL_ERROR, ("Failed to open report file for writing: %s", report_file_path));
    }
    free(report_str); // Liberar la memoria aquí
  } else {
    LOG(LL_ERROR, ("Failed to allocate memory for report string"));
  }
}


// ------------------------------------------------------- open_valve
bool open_valve() {
  valve_state = 1;
  mgos_gpio_write(LED_PIN, 1);
  mgos_gpio_write(VALVE_PIN, 1);
  mgos_gpio_write(LED_PIN, 1);
  json_str_msg = json_asprintf("{method: img, params: {name:\"%s\", pos_x:0, pos_y:208, size_x:32, size_y:32, back:0, front:1}}", "open_valve.raw");
  LOG(LL_INFO, ("OPEN VALVE"));
  mgos_uart_printf(UART_NO, "%s\n", json_str_msg);
  free(json_str_msg);  // Liberar memoria aquí
  json_str_msg = NULL;
  on_service = 1;
  timer_counter = 0;
  start_time = time(NULL);
  return true;
}

// ------------------------------------------------------- close_valve
bool close_valve() {
  valve_state = 0;
  mgos_gpio_write(LED_PIN, 0);
  mgos_gpio_write(VALVE_PIN, 0);
  mgos_gpio_write(LED_PIN, 0);
  json_str_msg = json_asprintf("{method: img, params: {name:\"%s\", pos_x:0, pos_y:208, size_x:32, size_y:32, back:1, front:0}}", "valve.raw");
  LOG(LL_INFO, ("CLOSE VALVE"));
  mgos_uart_printf(UART_NO, "%s\n", json_str_msg);
  free(json_str_msg);  // Liberar memoria aquí
  json_str_msg = NULL;
  return false;
}

// ------------------------------------------------------- save_repor_file
void save_report_file() {
  //create_folder(log_folder);

  char file_path[100];
  //snprintf(file_path, sizeof(file_path), "fol_%04d_%02d_%02d.json", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
  snprintf(file_path, sizeof(file_path), "rep_%lld.json", reporte);

  char start_time_str[20], end_time_str[20];
  strftime(start_time_str, sizeof(start_time_str), "%Y-%m-%d %H:%M:%S", localtime(&start_time));
  strftime(end_time_str, sizeof(end_time_str), "%Y-%m-%d %H:%M:%S", localtime(&end_time));

  /*char *json_str = json_asprintf(
    "{"
    "folio: %lld,"
    "start_time: \"%s\","
    "end_time: \"%s\","
    "start_position: %lld,"
    "end_position: %lld,"
    "current_pulses: %lld,"
    "pulsos_litro: %.2f,"
    "litros: %d,"
    "precio_litro: %.2f,"
    "precio: %d"
    "}\n",
    folio, start_time_str, end_time_str, start_service_position, end_service_position, current, pulsos_litro, litros, precio_litro, precio
  );*/
  char *json_str = json_asprintf(
    "{"
    "folio: %lld,"
    "pulsos_l: %.2f,"
    "litros: %d,"
    "precio_l: %.2f,"
    "precio: %d"
    "}\n",
    folio, pulsos_litro, litros, precio_litro, precio
  );
  
  LOG(LL_INFO, ("Servicio: %s", json_str));

  if (json_str != NULL) {
    FILE *f = fopen(file_path, "a");
    if (f != NULL) {
      fwrite(json_str, 1, strlen(json_str), f);
      fclose(f);
      LOG(LL_INFO, ("Saved to file: %s", file_path));
      
    } else {
      LOG(LL_ERROR, ("Failed to open file for writing: %s", file_path));
    }
    free(json_str);
  } else {
    LOG(LL_ERROR, ("Failed to allocate memory for JSON string"));
  }

}

// ------------------------------------------------------- save_to_file_in_folder
void save_to_file_in_folder() {
  //create_folder(log_folder);

  char file_path[100];
  //snprintf(file_path, sizeof(file_path), "fol_%04d_%02d_%02d.json", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
  snprintf(file_path, sizeof(file_path), "fol_%lld.json", folio);

  char start_time_str[20], end_time_str[20];
  strftime(start_time_str, sizeof(start_time_str), "%Y-%m-%d %H:%M:%S", localtime(&start_time));
  strftime(end_time_str, sizeof(end_time_str), "%Y-%m-%d %H:%M:%S", localtime(&end_time));

  char *json_str = json_asprintf(
    "{"
    "folio: %lld,"
    "s_time: \"%s\","
    "e_time: \"%s\","
    "s_pos: %lld,"
    "e_pos: %lld,"
    "pulses: %lld,"
    "pulsos_l: %.2f,"
    "litros: %d,"
    "precio_l: %.2f,"
    "precio: %d"
    "}\n",
    folio, start_time_str, end_time_str, start_service_position, end_service_position, current, pulsos_litro, litros, precio_litro, precio
  );
  
  LOG(LL_INFO, ("Servicio: %s", json_str));

  if (json_str != NULL) {
    FILE *f = fopen(file_path, "w");
    if (f != NULL) {
      fwrite(json_str, 1, strlen(json_str), f);
      fclose(f);
      LOG(LL_INFO, ("Saved to file: %s", file_path));
      print_tiket(json_str);
      char full_topic_str[128];
      snprintf(full_topic_str, sizeof(full_topic_str), "%s/out", topic_str);
      mgos_mqtt_pub(full_topic_str, json_str, strlen(json_str), 1, 0);  /* Publish */
    } else {
      LOG(LL_ERROR, ("Failed to open file for writing: %s", file_path));
    }
    free(json_str);
  } else {
    LOG(LL_ERROR, ("Failed to allocate memory for JSON string"));
  }

}
// -------------------------------------------------------- save position
static void save_position() {
  char *json_str = json_asprintf("{position: %lld, on_service: %d, start_position: %lld, end_position: %lld, folio: %lld, reporte: %lld}", position, on_service, start_position, end_position, folio, reporte);
  if (json_str != NULL) {
    // Leer el contenido actual del archivo
    FILE *f = fopen(filename, "r");
    char current_content[512]; // Asumiendo que el contenido no será mayor de 512 caracteres
    size_t len = 0;

    if (f != NULL) {
      len = fread(current_content, 1, sizeof(current_content) - 1, f);
      current_content[len] = '\0'; // Asegurarse de que la cadena esté terminada en null
      fclose(f);
    } else {
      current_content[0] = '\0'; // Si el archivo no existe, considerar que está vacío
    }

    // Comparar el contenido actual con el nuevo contenido
    if (strcmp(current_content, json_str) != 0) {
      // Si el contenido es diferente, escribir el nuevo contenido en el archivo
      f = fopen(filename, "w");
      if (f != NULL) {
        fwrite(json_str, 1, strlen(json_str), f);
        fclose(f);
        //LOG(LL_INFO, ("Saved to file: %s", json_str));
      } else {
        LOG(LL_ERROR, ("Failed to open file for writing"));
      }
    }

    free(json_str);
  } else {
    LOG(LL_ERROR, ("Failed to allocate memory for JSON string"));
  }
}

// ------------------------------------------------------------- check_delta
static void check_delta() {
  start_delta_time = mgos_uptime_micros();
  dif_delta_time = start_delta_time - end_delta_time;

  if(dif_delta_time < (delta_time/delta))
  {
    if(delta_pulses > delta)
    {
      // ------------------------- START
      mgos_gpio_write(LED_PIN,1);
      start_position = position - (position - last_position);
      start_service_position = start_position;
      current = 0;
      on_service = 1;
      start_time = time(NULL);
    }
    delta_pulses++;
  }
  else
    delta_pulses = 0;
  
  end_delta_time = start_delta_time;
  
}

// ------------------------------------------------------ updateEncoder
static void updateEncoder() {
  int MSB = mgos_gpio_read(ENCODER_PIN_A); // MSB = most significant bit
  int LSB = mgos_gpio_read(ENCODER_PIN_B); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Combinar los estados de los pines A y B
  int sum = (lastEncoded << 2) | encoded; // Combinar con el último estado

  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) {
    position++;
  } else if (sum == 14 || sum == 7 || sum == 1 || sum == 8) {
    position--;
  }

  if((on_service == 0))
  {
    check_delta();
  }
  
  lastEncoded = encoded; // Actualizar el último estado
  //mgos_gpio_write(LED_PIN,1);
  //LOG(LL_INFO, ("Encoder Position: %d", position));
  
}


// ----------------------------------------------------------- encoder_init
bool encoder_init() {
  mgos_gpio_set_mode(ENCODER_PIN_A, MGOS_GPIO_MODE_INPUT);
  mgos_gpio_set_mode(ENCODER_PIN_B, MGOS_GPIO_MODE_INPUT);

  mgos_gpio_set_pull(ENCODER_PIN_A, MGOS_GPIO_PULL_UP);
  mgos_gpio_set_pull(ENCODER_PIN_B, MGOS_GPIO_PULL_UP);

  mgos_gpio_set_int_handler(ENCODER_PIN_A, MGOS_GPIO_INT_EDGE_ANY, updateEncoder, NULL);
  mgos_gpio_set_int_handler(ENCODER_PIN_B, MGOS_GPIO_INT_EDGE_ANY, updateEncoder, NULL);

  mgos_gpio_enable_int(ENCODER_PIN_A);
  mgos_gpio_enable_int(ENCODER_PIN_B);

  return true;
}


// ------------------------------------------------------------- load_pos
static void load_position() {
  FILE *f = fopen(filename, "r");
  if (f != NULL) {
    char buffer[512];
    int len = fread(buffer, 1, sizeof(buffer) - 1, f);
    buffer[len] = '\0';
    fclose(f);
    json_scanf(buffer, len, "{position: %lld, start_position: %lld, end_position: %lld, on_service: %d, folio: %lld, reporte:%lld}", &position, &start_position, &end_position, &on_service, &folio, &reporte);

    if (on_service) {
      last_position = end_position;
      mgos_gpio_write(LED_PIN, 1);
    } else {
      last_position = position;
    }

    //save_position();
  } else {
    LOG(LL_ERROR, ("Failed to open file for reading"));
  }
}




// ------------------------------------------------------------- timer_delta
static void timer_delta(void *arg) {
  end_position = position;
  save_position();

  if (on_service == 1) 
  {
    LOG(LL_INFO, ("Service Started"));
    on_service = 2;
    

  }else if(on_service == 2)
  {
   if(abs(position - last_position) > delta)
   {
      last_dif_delta_time = mgos_uptime();
   }
   else
   { 
    static int64_t delta_micros;
    delta_micros = mgos_uptime();
    if ( (delta_micros - last_dif_delta_time) > (stop_time))
    {
      LOG(LL_INFO, ("Service STOP"));
      on_service = 0;
        delta_pulses = 0;
        start_position = end_position;
        end_service_position = end_position;
        close_valve();
        end_time = time(NULL);
        if(litros > 0)
        {
          save_to_file_in_folder(); // Save data to file when service ends
          save_report_file();
          last_json_str_msg = NULL;
          current = 0;
          folio++;
        }
    }
   }
    
   /* if ( (mgos_uptime_micros() - last_dif_delta_time) > (stop_time* 1000)) 
    {
      
        //last_dif_delta_time = mgos_uptime_micros();
        // ------------------------- END
        on_service = 0;
        delta_pulses = 0;
        start_position = end_position;
        end_service_position = end_position;
        close_valve();
        end_time = time(NULL);
        if(litros > 0)
        {
          save_to_file_in_folder(); // Save data to file when service ends
          save_report_file();
          last_json_str_msg = NULL;
          current = 0;
          folio++;
        }
        
      
    }*/
  }
  
  if (on_service > 0)
  {
    current = abs(end_position - start_position);
    //if(current > 0)
    litros = current / pulsos_litro;
    precio = litros * precio_litro;
  }
  
  else {
    current = 0;
    litros = 0;
    precio = 0;
  }

  send_to_display();
  last_position = position;
  (void)arg;
}



// UART handler to process received data
static void uart_dispatcher(int uart_no, void *arg) {
  static struct mbuf recv_mbuf;
  static bool initialized = false;

  if (!initialized) {
    mbuf_init(&recv_mbuf, 256);
    initialized = true;
  }

  size_t rx_avail = mgos_uart_read_avail(uart_no);
  if (rx_avail > 0) {
    // Allocate buffer for the received data
    char *data = (char *)malloc(rx_avail);
    int len = mgos_uart_read(uart_no, data, rx_avail);
    mbuf_append(&recv_mbuf, data, len);
    free(data); // Liberar la memoria aquí

    // Check if we have a full line
    char *newline = strchr(recv_mbuf.buf, '\n');
    if (newline != NULL) {
      *newline = '\0'; // Null-terminate the line

      // Process the JSON data
      struct json_token method_token = JSON_INVALID_TOKEN;
      struct json_token key_token = JSON_INVALID_TOKEN;
      json_scanf(recv_mbuf.buf, recv_mbuf.len, "{method: %T, params: {key: %T}}", &method_token, &key_token);

      if (method_token.len > 0 && strncmp(method_token.ptr, "key_press", method_token.len) == 0 && key_token.len > 0) {
        char key[2];
        strncpy(key, key_token.ptr, key_token.len);
        key[key_token.len] = '\0';
        LOG(LL_INFO, ("key: %s", key));

        // Add your logic to handle key press here
        if (strcmp(key, "*") == 0) {
          close_valve();
          // Implement your action for key '1' press
        }

        if (strcmp(key, "#") == 0) {
          open_valve();
          // Implement your action for key '1' press
        }

        if (strcmp(key, "A") == 0) {
          make_report();
          // Implement your action for key '1' press
        }
      }

      // Remove the processed line from the buffer
      mbuf_remove(&recv_mbuf, newline - recv_mbuf.buf + 1);
    }
  }
  (void)arg;
}

// UART handler to process received data for UART_NO1
static void uart_dispatcher1(int uart_no, void *arg) {
  static struct mbuf recv_mbuf1;
  static bool initialized1 = false;

  if (!initialized1) {
    mbuf_init(&recv_mbuf1, 1500);
    initialized1 = true;
  }

  size_t rx_avail = mgos_uart_read_avail(uart_no);
  if (rx_avail > 0) {
    // Allocate buffer for the received data
    char *data = (char *)malloc(rx_avail);
    int len = mgos_uart_read(uart_no, data, rx_avail);
    mbuf_append(&recv_mbuf1, data, len);
    free(data); // Liberar la memoria aquí

    // Check if we have a full line
    char *newline = strchr(recv_mbuf1.buf, '\n');
    if (newline != NULL) {
      *newline = '\0'; // Null-terminate the line

      // Process the JSON data
      struct json_token method_token = JSON_INVALID_TOKEN;
      struct json_token key_token = JSON_INVALID_TOKEN;
      json_scanf(recv_mbuf1.buf, recv_mbuf1.len, "{method: %T, params: {key: %T}}", &method_token, &key_token);

      if (method_token.len > 0 && strncmp(method_token.ptr, "key_press", method_token.len) == 0 && key_token.len > 0) {
        char key[2];
        strncpy(key, key_token.ptr, key_token.len);
        key[key_token.len] = '\0';
        LOG(LL_INFO, ("key: %s", key));

        // Add your logic to handle key press here
        if (strcmp(key, "*") == 0) {
          close_valve();
          // Implement your action for key '1' press
        }

        if (strcmp(key, "#") == 0) {
          open_valve();
          // Implement your action for key '1' press
        }

        if (strcmp(key, "A") == 0) {
          make_report();
          // Implement your action for key '1' press
        }
      }

      // Remove the processed line from the buffer
      mbuf_remove(&recv_mbuf1, newline - recv_mbuf1.buf + 1);
    }
  }
  (void)arg;
}

// Inicializar -------------------------------- UART
static void uart_init() {
  struct mgos_uart_config ucfg;

  // Configurar resistencias pull-up/pull-down en los pines RX y TX del UART
  mgos_gpio_set_pull(16, MGOS_GPIO_PULL_UP);
  mgos_gpio_set_pull(17, MGOS_GPIO_PULL_DOWN);
  mgos_uart_config_set_defaults(UART_NO, &ucfg);

  ucfg.baud_rate = 115200;
  ucfg.rx_buf_size = 1500;
  ucfg.tx_buf_size = 1500;
 
  if (!mgos_uart_configure(UART_NO, &ucfg)) {
    LOG(LL_ERROR, ("Failed to configure UART%d", UART_NO));
  } else {
    mgos_uart_set_dispatcher(UART_NO, uart_dispatcher, NULL);
    mgos_uart_set_rx_enabled(UART_NO, true);
  }


  //mgos_gpio_set_pull(16, MGOS_GPIO_PULL_UP);
  //mgos_gpio_set_pull(17, MGOS_GPIO_PULL_DOWN);
  mgos_uart_config_set_defaults(UART_NO1, &ucfg);
  ucfg.baud_rate = 38400;
  ucfg.rx_buf_size = 1500;
  ucfg.tx_buf_size = 1500;

  if (!mgos_uart_configure(UART_NO1, &ucfg)) {
    LOG(LL_ERROR, ("Failed to configure UART%d", UART_NO1));
  } else {
    mgos_uart_set_dispatcher(UART_NO1, uart_dispatcher1, NULL);
    mgos_uart_set_rx_enabled(UART_NO1, true);
  }

  // Reasignar pines para UART1
  
 // uart_set_pin(UART_NO1, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Inicialización del ------------------------ DS3231
bool ds3231_init(void) {
  rtc = mgos_ds3231_create(MGOS_DS3231_DEFAULT_I2C_ADDR);
  if (rtc == NULL) {
    LOG(LL_ERROR, ("Failed to initialize DS3231"));
    return false;
  }
  else
  {
    LOG(LL_INFO, ("RTC INIT OK"));
    //const struct mgos_ds3231_date_time *dt;
    //dt = mgos_ds3231_read(rtc);
    mgos_ds3231_settimeofday(rtc);
    return true;
  }
  
}


// ------------------------- Clear_screen
void clear_screen() {
  static char *json_str_msg = NULL;
  json_str_msg = json_asprintf("{method: clear_screen, params: {clear: true}}");
  mgos_uart_printf(UART_NO, "%s\n", json_str_msg);
  free(json_str_msg); // Liberar la memoria aquí
}



//############################################ INIT ###############################
enum mgos_app_init_result mgos_app_init(void) {
  ENCODER_PIN_A = mgos_sys_config_get_app_pinA();
  ENCODER_PIN_B = mgos_sys_config_get_app_pinB();
  LED_PIN = mgos_sys_config_get_app_LED_PIN();
  VALVE_PIN = mgos_sys_config_get_app_VALVE_PIN();
  pulsos_litro = mgos_sys_config_get_app_pulsos_litro();
  precio_litro = mgos_sys_config_get_app_precio_litro();
  stop_time = mgos_sys_config_get_app_stop_time();
  delta_time = mgos_sys_config_get_app_delta_time();
  topic_str = mgos_sys_config_get_app_id();

  mgos_gpio_setup_output(LED_PIN, 0);
  mgos_gpio_write(LED_PIN, 0);
  mgos_gpio_setup_output(VALVE_PIN, 0);
  mgos_gpio_write(VALVE_PIN, 0);

  start_delta_time = mgos_uptime();
  end_delta_time = start_delta_time;

  encoder_init();
  load_position(); // Cargar la posición desde el archivo al iniciar
  uart_init();     // Inicializar UART
  clear_screen();
  close_valve();
  //timer_display(NULL);

  

  // Inicializar el DS3231 // Inicializar el DS3231
  if (!ds3231_init()) {
    LOG(LL_ERROR, ("Failed to initialize DS3231"));
    return MGOS_APP_INIT_ERROR;
  }

  //create_folder(report_folder);
  if (topic_str != NULL) {
        char full_topic_str[128];
        snprintf(full_topic_str, sizeof(full_topic_str), "%s/in", topic_str);
        //mgos_mqtt_sub(full_topic_str, handler, NULL); /* Subscribe */
        //LOG(LL_INFO, ("Subscribed to topic: %s", full_topic_str));
    } else {
        LOG(LL_ERROR, ("topic_str is NULL"));
    }

mgos_set_timer(delta_time /* ms */, MGOS_TIMER_REPEAT, timer_delta, NULL);
 return MGOS_APP_INIT_SUCCESS;
}