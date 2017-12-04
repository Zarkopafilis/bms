////DUMP - ONLY APPEND TO THIS FILE
//
////Broadcast new configuration
//  //REFON=1 -> Always awake
//  //VUV (Undervoltage) & VOV (Overvoltage) Values
//  //WRCFG (Write Configuration) Command
//  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
//    output_low(SS);
//
//    for(uint8_t i = 0; i < 4; i++){
//      spi_write(write_cfg_cmd_tx[i]);
//    }
//
//    for(uint8_t i = 0; i < 8; i++){
//      spi_write(write_cfg_data_tx[i]);
//    }
//
//    output_high(SS);
//
//    #ifdef DEBUG
//      Serial.print("Slave ");
//      Serial.print(addr);
//      Serial.println(" configuration write attempt");
//    #endif
//
//
//
//      //Check if every slave is connected and is able to communicate properly
//  //RDCFG (Read Configuration) Command
//  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
//    output_low(SS);
//
//    for(uint8_t i = 0; i < 4; i++){
//      spi_write(read_cfg_tx[i]);
//    }
//
//    for (uint8_t i = 0; i < TOTALDATABYTES; i++){
//      read_cfg_rx[i] = spi_read(0);
//    }
//
//    output_high(SS);
//
//    #ifdef DEBUG
//      Serial.print("Slave ");
//      Serial.print(addr);
//      Serial.println(" connected");
//    #endif
//
//
//
//
//      for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
//    uint8_t check = 0xFF;
//    output_low(SS);
//
//    for(uint8_t i = 0; i < 4; i++){
//      spi_write(read_cfg_tx[i]);
//    }
//
//    for (uint8_t i = 0; i < TOTALDATABYTES; i++){
//      uint8_t byte = spi_read(0);
//      check &= (byte == write_cfg_data_tx[i]);
//    }
//
//    output_high(SS);
//
//    if(check == 0xFF){
//      #ifdef DEBUG
//        Serial.print("Slave ");
//        Serial.print(addr);
//        Serial.println(" got properly configured");
//      #endif
//    }else{
//      #ifdef DEBUG
//        Serial.print("Slave ");
//        Serial.print(addr);
//        Serial.println(" was not properly configured!");
//      #endif
//      shut_car_down();
//    }
// }
