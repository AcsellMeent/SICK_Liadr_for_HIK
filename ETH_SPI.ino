#include <cola2.h>

#include <algorithm>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "rpLidar.h"
#include "rpLidarTypes.h"

#define W5500_CS   10
#define W5500_SCK  12
#define W5500_MISO 13
#define W5500_MOSI 11

rpLidar lidar(&Serial1,1000000);

byte mac[] = {
  0x02, 0x12, 0x34, 0x56, 0x78, 0x9A
};

IPAddress ip(192, 168, 1, 85);
IPAddress dns(8, 8, 8, 8);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

IPAddress host(192, 168, 1, 64);

EthernetUDP udp;

EthernetServer server(2122);

const uint16_t LOCAL_PORT = 6060;
const uint16_t REMOTE_PORT = 6060;

const uint16_t max_udp_payload_size = 1460;
const size_t MEASUREMENT_DATA_SAMPLE_SIZE = 4;

uint8_t datagram_header[24] = { 0x4d, 0x53, 0x33, 0x20, 0x4d, 0x44, 0x1, 0x0, 0xE8, 0x3, 0x0, 0x0, 0xe9, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

uint8_t data_output_header[56] = { 0x52, 0x2, 0x0, 0x0, 0xdc, 0x84, 0x70, 0x1, 0x31, 0x41, 0x56, 0x1, 0x0, 0x0, 0x0, 0x0, 0xe9, 0xf, 0x0, 0x0, 0xd8, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x78, 0xd9, 0x1, 0x0, 0x4c, 0x0, 0x10, 0x0, 0x60, 0x0, 0x18, 0x0, 0x7c, 0x0, 0xC0, 0x3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x0, 0x0, 0x0 };

uint8_t device_status_block[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
// uint8_t device_status_block[16] = { 0x1, 0x1, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0 };

uint8_t configuration_block[24] = { 0x1, 0x0, 0xC0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x20, 0xF4, 0x9A, 0x99, 0x11, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

uint32_t sequence_number = 0;

uint8_t udp_package[max_udp_payload_size];

uint8_t datagram_header_size;
uint8_t data_output_header_size; 
uint8_t device_status_block_size;
uint8_t configuration_block_size;
uint8_t static_data_size;

const size_t LENGHT_FOR_SEND = 1651;

void setup()
{
  Serial.begin(115200);

  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);

  Ethernet.init(W5500_CS);

  Ethernet.begin(mac, ip, gateway, gateway, subnet);

  delay(1000);

  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  server.begin();

  udp.begin(LOCAL_PORT);

  lidar.resetDevice();
  // lidar.setAngleOfInterest(0,360, false);
  lidar.setAngleOfInterest(222.5, 137.5, true);
  lidar.start(express);

  Serial.print("UDP port: ");
  Serial.println(LOCAL_PORT);

  datagram_header_size = sizeof(datagram_header)/sizeof(uint8_t);
  data_output_header_size = sizeof(data_output_header)/sizeof(uint8_t);
  device_status_block_size = sizeof(device_status_block)/sizeof(uint8_t);
  configuration_block_size = sizeof(configuration_block)/sizeof(uint8_t);
  // configuration_block_size = 0;

  static_data_size = datagram_header_size + data_output_header_size + device_status_block_size + configuration_block_size;
}

void loop()
{
  CoLaConnect(server);

  receiveTCP();

  if (ctx.session.active)
  {
    if (millis() -
      ctx.session.lastActivity >
      ctx.session.timeoutMs)
    {
      Serial.println(
        "CoLa session timeout"
      );

      ctx.session.active =
       false;
    }
  }

  if (ctx.client &&
    !ctx.client.connected())
  {
    Serial.println(
      "CoLa client disconnected"
    );

    ctx.client.stop();

    ctx.rxLength = 0;

    ctx.session.active =
      false;
  }

  if (outputEnabled) {
    
    size_t actual_lenght = lidar.readMeasurePoints();

    if(actual_lenght == 0) {
      return;
    }

    int slice_point = 0;

    std::vector<point_t> trimed_data(LENGHT_FOR_SEND);

    std::vector<point_t> data;
    size_t number_of_beams = 0;

    for(int i = SCAN_DATA_SIZE - actual_lenght; i < SCAN_DATA_SIZE; i++) {
      number_of_beams ++;
      data.push_back(lidar.Data[i]);
    }

    // Serial.println(actual_lenght);

    // double delta = (double)275 / (actual_lenght - 1);
    // for(int i=0; i < actual_lenght; i ++) {
    //     data[i].angle = i*delta;ы
    // }

    // double step = (double)data.size() / trimed_data.size();
    // size_t index = 0;
    // double i = 0;

    // while (round(i) < data.size()) {
    //     i += step;
    //     trimed_data[index++] = data[round(i)];
    // };

    double angle_delta = (double)275.0 / ((double)LENGHT_FOR_SEND - 1);

    int input_data_index = 1;
    int start_sector_index = 0;

    double actual_angle = 0;

    Serial.println();

    for(int i = 0; i < LENGHT_FOR_SEND; i ++) {
      double prev_angle = 0;
      uint16_t distance = 0;

      trimed_data[i].angle = (i * angle_delta);

      if (slice_point == 0 && trimed_data[i].angle > 137.5) {
        slice_point = i;
      }

      if (slice_point != 0) {
        trimed_data[i].angle += 85.0;
      }

      if(input_data_index == actual_lenght) {
        break;
      }
      
      while(data[input_data_index].angle <= (trimed_data[i].angle) + angle_delta/2) {
        input_data_index++;
      }
      if(data[input_data_index-1].angle > (trimed_data[i].angle) - angle_delta/2) {
        distance = data[input_data_index-1].distance;
        if(trimed_data[i].angle > 90 && trimed_data[i].angle < 137.5) {
          Serial.printf("angle %f index %d dista %d\n", data[input_data_index-1].angle, input_data_index-1, distance);
        }
      }

      trimed_data[i].distance = distance;
    }



    // double angle_delta = (double)275.0 / ((double)LENGHT_FOR_SEND - 1);

    // int input_data_index = 0;
    // int start_sector_index = 0;

    // double actual_angle = 0;

    // for(int i = 0; i < LENGHT_FOR_SEND; i ++) {
    //   uint32_t sum_distance = 0;
    //   uint16_t sum_lenght = 0;
    //   start_sector_index = input_data_index;

    //   trimed_data[i].angle = (i * angle_delta);

    //   if (slice_point == 0 && trimed_data[i].angle > 137.5) {
    //     slice_point = i;
    //   }

    //   if (slice_point != 0) {
    //     trimed_data[i].angle += 85.0;
    //   }

    //   // Serial.println();
    //   // Serial.printf("dista %d\n", data[input_data_index].distance);
    //   // Serial.printf("sum %d lenght %d start %d end %d\n", sum_distance, sum_lenght, start_sector_index, input_data_index);
    //   // Serial.printf("angle %f start %f end %f\n", trimed_data[i].angle, data[start_sector_index].angle, data[input_data_index].angle);
      
    //   while(data[input_data_index].angle <= (trimed_data[i].angle) + angle_delta/2) {
    //     // Serial.printf("dista %d\n", data[input_data_index].distance);

    //     if(data[input_data_index].distance == 0) {
    //       input_data_index++;
    //       continue;
    //     }
    //     sum_distance = data[start_sector_index].distance;
    //     sum_lenght = 1;
    //     input_data_index++;
        
    //   }

    //   if(input_data_index - start_sector_index == 0) {
    //     trimed_data[i].distance = 0;
    //     continue;
    //   }
    //   if(sum_lenght != 0) {
    //     trimed_data[i].distance = uint16_t(sum_distance / sum_lenght);
    //     // Serial.printf("angle %f dista %d\n", trimed_data[i].angle, trimed_data[i].distance);
    //   }
    //   else{
    //     trimed_data[i].distance = 0;
    //   }


    //   // if(trimed_data[i].distance != 40000) {
    //   // Serial.printf("res %d\n", trimed_data[i].distance);

    //     // Serial.printf("sum %d lenght %d start %d end %d\n", sum_distance, sum_lenght, start_sector_index, input_data_index);
    //     // Serial.printf("angle %f start %f end %f\n", trimed_data[i].angle, data[start_sector_index].angle, data[input_data_index].angle);

    //   // }
    //   // if(sum_lenght > 2) {
    //     // Serial.println();
    //     // Serial.printf("sum %d lenght %d start %d end %d\n", sum_distance, sum_lenght, start_sector_index, input_data_index);
    //     // Serial.printf("angle %f start %f end %f\n", trimed_data[i].angle, data[start_sector_index].angle, data[input_data_index].angle);
    //   // }
    // }
    
    std::reverse(trimed_data.begin(), trimed_data.begin() + slice_point);
    std::reverse(trimed_data.begin() + slice_point, trimed_data.end());

    number_of_beams = LENGHT_FOR_SEND;

    uint16_t total_measurement_data_size = (number_of_beams * 4) + 4;

    uint32_t total_size = static_data_size - datagram_header_size + total_measurement_data_size + 28; // 28 is a non-specified linght

    uint16_t measurement_data_start_index = 0;

    writeUint32LittleEndian(datagram_header + 8, total_size);

    uint8_t packets_count = 0;

    if(static_data_size + total_measurement_data_size <= 1460) {
      packets_count = 1;
    } else {
      packets_count = ((static_data_size + total_measurement_data_size) + max_udp_payload_size-1 ) / max_udp_payload_size;
    }

    uint16_t fragment_offset = 0;

    uint8_t out_sample[MEASUREMENT_DATA_SAMPLE_SIZE]{};
    uint16_t out_sample_size = 0;

    size_t write_size = 0;

    uint16_t total_measurement_data_offset = 0;

    for (int p = 0; p < packets_count; p ++) {
      memset(udp_package, 0, max_udp_payload_size);

      uint16_t measurement_data_offset = datagram_header_size;

      // Headers setup

      memcpy(udp_package, datagram_header, datagram_header_size);

      fragment_offset = p * (max_udp_payload_size - 24);
      writeUint32LittleEndian(udp_package + 16, fragment_offset);
      writeUint32LittleEndian(udp_package + 12, sequence_number);

      if(p == 0) {
        // measurement_data_offset = 128; // without conf
        measurement_data_offset = 152;

        writeUint32LittleEndian(data_output_header + 16, sequence_number);
        writeUint32LittleEndian(data_output_header + 20, sequence_number);
        writeUint32LittleEndian(data_output_header + 28, millis());
        writeUint16LittleEndian(data_output_header + 36, 96);
        writeUint16LittleEndian(data_output_header + 38, configuration_block_size);
        writeUint16LittleEndian(data_output_header + 40, measurement_data_offset - datagram_header_size - 4);
        writeUint16LittleEndian(data_output_header + 42, total_measurement_data_size);
        
        writeUint16LittleEndian(configuration_block + 2, number_of_beams);
        writeUint16LittleEndian(configuration_block + 4, millis());
        writeUint32LittleEndian(configuration_block + 8, 47.5);
        writeUint32LittleEndian(configuration_block + 12, (uint32_t)((275.0 / number_of_beams) * 4194304));
        // memset(configuration_block, 0, configuration_block_size);

        memcpy(udp_package + datagram_header_size, data_output_header, data_output_header_size);
        memcpy(udp_package + 100, device_status_block, device_status_block_size);

        memcpy(udp_package + 120, configuration_block, configuration_block_size);
        
        writeUint16LittleEndian(udp_package + measurement_data_offset - 4, number_of_beams);

        // writeUint32LittleEndian(udp_package + 132, (uint32_t)((275.0 / number_of_beams) * 4194304));
      }

      // Headers setup

      // Measurement data wire

      uint16_t frame_measurement_data_size = 0;

      if(packets_count > 1) {
        frame_measurement_data_size = (max_udp_payload_size - measurement_data_offset);
        if((number_of_beams*4) - measurement_data_start_index < frame_measurement_data_size) {
          frame_measurement_data_size = (number_of_beams*4) - measurement_data_start_index;
        }
      } else {
        frame_measurement_data_size = total_measurement_data_size - 4;
      }
      
      measurement_data_start_index += frame_measurement_data_size;

      uint16_t write_offset = measurement_data_offset;
        
      if (out_sample_size != 0) {
        memcpy(udp_package + write_offset, out_sample + out_sample_size, MEASUREMENT_DATA_SAMPLE_SIZE - out_sample_size); 
        write_offset += MEASUREMENT_DATA_SAMPLE_SIZE - out_sample_size;
        out_sample_size = 0;
      }
        
      write_size = max_udp_payload_size - write_offset;
      
      writeMeasurementData(udp_package + write_offset, trimed_data.begin() + total_measurement_data_offset, write_size, out_sample, out_sample_size);
      
      total_measurement_data_offset += std::ceil(write_size / (double)MEASUREMENT_DATA_SAMPLE_SIZE);

      // Measurement data wire

      // UDP send
      
      udp.beginPacket(host, REMOTE_PORT);

      udp.write(udp_package, measurement_data_offset + frame_measurement_data_size);

      // Serial.printf("Sended bytes: %d\n", measurement_data_offset + frame_measurement_data_size);
    
      int result = udp.endPacket();

      if (result == 1) {
        // Serial.println("UDP sent");
      } else {
        // Serial.println("UDP send ERROR");
      }
      
      // UDP send

    }
    sequence_number ++;
  }

}

void writeMeasurementData(uint8_t* buffer, std::vector<point_t>::iterator data, size_t write_size, uint8_t* out_sample, uint16_t &out_sample_size)
{
    size_t num_of_iteration = write_size / MEASUREMENT_DATA_SAMPLE_SIZE;
    
    if (std::ceil(write_size / (double)MEASUREMENT_DATA_SAMPLE_SIZE) > num_of_iteration) {
        out_sample_size = write_size % MEASUREMENT_DATA_SAMPLE_SIZE;
        out_sample[2] = 191;
        out_sample[3] = 1;
        writeUint16LittleEndian(out_sample, data[num_of_iteration].distance);
        memcpy(buffer + write_size - out_sample_size, out_sample, out_sample_size);
    }
    
    for (int i = 0; i < num_of_iteration; i++) {
        uint8_t sample[MEASUREMENT_DATA_SAMPLE_SIZE]{0,0,191,1};
        writeUint16LittleEndian(sample, data[i].distance);
        memcpy(buffer + i * MEASUREMENT_DATA_SAMPLE_SIZE, sample, MEASUREMENT_DATA_SAMPLE_SIZE);
    }   
}

void writeUint32LittleEndian(uint8_t* dst, uint32_t value)
{
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
    dst[2] = (value >> 16) & 0xFF;
    dst[3] = (value >> 24) & 0xFF;
}

void writeUint16LittleEndian(uint8_t* dst, uint16_t value) {
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
}
