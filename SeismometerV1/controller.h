struct DateTime{
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t year_bits;
    uint8_t month_bits;
    uint8_t date_bits;
    uint8_t hour_bits;
    uint8_t minute_bits;
    uint8_t second_bits;

};

//******************
// clock
void init_clock(void);
void write_to_clock( uint8_t address, uint8_t val);
uint8_t read_from_clock( uint8_t address);
void write_datetime_to_clock(struct DateTime data);
struct DateTime read_datetime_from_clock(void);

void convert_bits_to_datetime( struct DateTime *dt );
void convert_datetime_to_bits( struct DateTime *dt );
