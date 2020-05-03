void gps_setup(void)
{

    Serial2.begin(9600);
    delay(250);

    //Disable GPGSV messages by using the ublox protocol.
    uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    Serial2.write(Disable_GPGSV, 11);
    delay(350); //A small delay is added to give the GPS some time to respond @ 9600bps.
    //Set the refresh rate to 5Hz by using the ublox protocol.
    uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    Serial2.write(Set_to_5Hz, 14);
    delay(350); //A small delay is added to give the GPS some time to respond @ 9600bps.
    //Set the baud rate to 57.6kbps by using the ublox protocol.
    uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
    Serial2.write(Set_to_57kbps, 28);
    delay(200);

    Serial2.begin(57600);
    delay(200);
}

void read_gps(void)
{
    while (Serial2.available() && new_line_found == 0)
    {
        char read_serial_byte = Serial2.read();
        if (read_serial_byte == '$')
        {
            for (message_counter = 0; message_counter <= 99; message_counter++)
            {
                incomming_message[message_counter] = '-';
            }
            message_counter = 0;
        }
        else if (message_counter <= 99)
            message_counter++;
        incomming_message[message_counter] = read_serial_byte;
        if (read_serial_byte == '*')
            new_line_found = 1;
    }

    //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
    if (new_line_found == 1)
    {
        new_line_found = 0;
        if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',')
        {
            digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));
            //Set some variables to 0 if no valid information is found by the GPS module.
            l_lat_gps = 0;
            l_lon_gps = 0;
            lat_gps_previous = 0;
            lon_gps_previous = 0;
            number_used_sats = 0;
        }
        //If the line starts with GA and if there is a GPS fix, scan the line for the latitude, longitude and number of satellites.
        if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2'))
        {
            lat_gps_actual = ((int)incomming_message[19] - 48) * (long)10000000;
            lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;
            lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;
            lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;
            lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;
            lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;
            lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;
            lat_gps_actual /= (long)6;
            lat_gps_actual += ((int)incomming_message[17] - 48) * (long)100000000;
            lat_gps_actual += ((int)incomming_message[18] - 48) * (long)10000000;
            lat_gps_actual /= 10;

            lon_gps_actual = ((int)incomming_message[33] - 48) * (long)10000000;
            lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;
            lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;
            lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;
            lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;
            lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;
            lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;
            lon_gps_actual /= (long)6;
            lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;
            lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;
            lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;
            lon_gps_actual /= 10;

            if (incomming_message[28] == 'N')
                latitude_north = 1;
            else
                latitude_north = 0;

            if (incomming_message[42] == 'E')
                longiude_east = 1;
            else
                longiude_east = 0;

            number_used_sats = ((int)incomming_message[46] - 48) * (long)10; //Filter the number of satillites from the GGA line.
            number_used_sats += (int)incomming_message[47] - 48;

            if (lat_gps_previous == 0 && lon_gps_previous == 0)
            {
                lat_gps_previous = lat_gps_actual;
                lon_gps_previous = lon_gps_actual;
            }

            lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;
            lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;

            l_lat_gps = lat_gps_previous;
            l_lon_gps = lon_gps_previous;

            lat_gps_previous = lat_gps_actual;
            lon_gps_previous = lon_gps_actual;

            //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
            gps_add_counter = 5;
            new_gps_data_counter = 9;
            lat_gps_add = 0;
            lon_gps_add = 0;
            new_gps_data_available = 1;
        }

        //If the line starts with SA and if there is a GPS fix, scan the line for the fix type (none, 2D or 3D).
        if (incomming_message[4] == 'S' && incomming_message[5] == 'A')
            fix_type = (int)incomming_message[9] - 48;
    }

    //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
    if (gps_add_counter == 0 && new_gps_data_counter > 0)
    {
        new_gps_data_available = 1;
        new_gps_data_counter--;
        gps_add_counter = 5;

        lat_gps_add += lat_gps_loop_add;
        if (abs(lat_gps_add) >= 1)
        {
            l_lat_gps += (int)lat_gps_add;
            lat_gps_add -= (int)lat_gps_add;
        }

        lon_gps_add += lon_gps_loop_add;
        if (abs(lon_gps_add) >= 1)
        {
            l_lon_gps += (int)lon_gps_add;
            lon_gps_add -= (int)lon_gps_add;
        }
    }

    if (new_gps_data_available)
    {
        if (number_used_sats < 8)
            digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));
        else
            digitalWrite(STM32_board_LED, LOW);
        gps_watchdog_timer = millis();
        new_gps_data_available = 0;
    }