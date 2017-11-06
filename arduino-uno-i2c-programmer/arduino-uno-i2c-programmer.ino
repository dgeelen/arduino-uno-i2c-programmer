/**
 * Wiring connections:
 *  Arduino  | VGA Port
 * ----------+---------
 *  SDA (20) | Pin 12
 *  SCL (21) | Pin 15
 *  +5V      | Pin 9
 *  GND      | Pin 5
 *
 * Note: i2c EDID port address = 0x50;
 */

#include <Wire.h>
#include <stdint.h>

uint8_t buffer[BUFFER_LENGTH] = {}; // 128 uint8_t EEPROM data buffer
bool repaint_menu( true );
int target_port( 0x50 );

enum wire_result {
	wire_result_ok = 0,
};

enum record_type {
	record_type_data                     = 0x00,
	record_type_end_of_file              = 0x01,
	record_type_extended_segment_address = 0x02,
	record_type_start_segment_address    = 0x03,
	record_type_extended_linear_address  = 0x04,
	record_type_start_linear_address     = 0x05,
};

uint8_t low_byte( short s ) {
	return ( s >> 0 ) & 0xff;
}

uint8_t high_byte( short s ) {
	return ( s >> 8 ) & 0xff;
}

void print_hex_byte( uint8_t b, bool prefix = true ) {
	if( prefix ) {
		Serial.print( "0x" );
	}
	if( b < 0x10 ) {
		Serial.print( '0' );
	}
	Serial.print( b, HEX );
}

void print_hex_word( uint16_t b, bool prefix = true ) {
	print_hex_byte( high_byte(b), prefix );
	print_hex_byte( low_byte(b), false );
}

void print_serial_error( uint8_t error ) {
	Serial.print( "Serial error: " );
	switch( error ) {
		case 0: Serial.print( "success" ); break;
		case 1: Serial.print( "data too long to fit in transmit buffer" ); break;
		case 2: Serial.print( "received NACK on transmit of address" ); break;
		case 3: Serial.print( "received NACK on transmit of data" ); break;
		default: {
			Serial.print( "other error (" );
			Serial.print( error, DEC );
			Serial.print( ")" );
		}; break;
	}
	Serial.println();
}

#define DEBUG_PRINT( x ) do { \
		Serial.print( #x ": " ); \
		Serial.print( (x), HEX ); \
		Serial.println(); \
	} while( 0 );

void ddc_print( unsigned int address, bool intel_hex ) {
	const short bytes_per_row = 0x10;
	const short rows = BUFFER_LENGTH / bytes_per_row;
	for( short row = 0; row < rows; ++row ) {
		//DEBUG_PRINT( row );
		//DEBUG_PRINT( rows );
		//DEBUG_PRINT( BUFFER_LENGTH );

		// Start code
		if( intel_hex )
			Serial.print( ':' );

		// uint8_t count
		if( intel_hex )
			print_hex_byte( bytes_per_row, false );

		// Address
		print_hex_word( address + bytes_per_row*row, intel_hex );

		if( !intel_hex )
			Serial.print( " | " );

		// Record type
		if( intel_hex )
			print_hex_byte( record_type_data, false );

		// Data
		uint8_t checksum = 0;
		for( short i = bytes_per_row*row; i < bytes_per_row*row + bytes_per_row; ++i ) {
			uint8_t b = buffer[i];
			checksum += b;
			print_hex_byte( b, false );
			if( !intel_hex )
				Serial.print( ' ' );
		}
		checksum = (~checksum) + 1;

		// data, again
		if( !intel_hex ) {
			Serial.print( "| " );
			for( short i = bytes_per_row*row; i < bytes_per_row*row + bytes_per_row; ++i ) {
				char c = buffer[i];
				if( c < ' ' || c > '~' )
					c = '.';
				Serial.print( c );
			}
		}
		else { // intel_hex, Checksum
			print_hex_byte( checksum, HEX );
		}

		// Next line
		Serial.print( '\n' );
	}

	// Write end-of-file record
	if( intel_hex )
		Serial.println( ":00000001FF" );
}

void ddc_read( int address ) {
	Serial.print( "Reading from address " );
	print_hex_word( address );
	Serial.println( "..." );

	Wire.beginTransmission( address );
	Wire.write( 0 ); // initial address (?)
	if( uint8_t error = Wire.endTransmission() != 0 ) {
		print_serial_error( error );
		return;
	}

	const int blocks = 128 / BUFFER_LENGTH;
	for(int block = 0; block < blocks; block += BUFFER_LENGTH) {
		Wire.requestFrom( address, BUFFER_LENGTH );
		for(int i = 0; i < BUFFER_LENGTH; i++) {
			buffer[block + i] = Wire.read();
			digitalWrite(LED_BUILTIN, i & 8 ? HIGH : LOW);
		}
	}

	digitalWrite(LED_BUILTIN, LOW);
	Serial.println("Finished reading.");
}

void dump_all_data( bool intel_hex ) {
	Serial.print( "Dumping all data from address " );
	print_hex_word( target_port );
	Serial.println( "..." );

	Wire.beginTransmission( target_port );
	Wire.write( 0 ); // initial address (?)
	if( uint8_t error = Wire.endTransmission() != 0 ) {
		print_serial_error( error );
		return;
	}

	int offset( 0 );
	while( 1 ) {
		Wire.requestFrom( target_port, BUFFER_LENGTH );
		for(int i = 0; i < BUFFER_LENGTH; i++) {
			buffer[i] = Wire.read();
			digitalWrite(LED_BUILTIN, i&8 ? HIGH : LOW);
		}
		ddc_print( offset, intel_hex );
		offset += BUFFER_LENGTH;
	}

	digitalWrite(LED_BUILTIN, LOW);
	Serial.println("Finished reading.");
}

void ddc_scan() {
	Serial.println("Scanning i2c bus...");

	int nr_found( 0 );
	// addresses 0..7 are not valid according to Wire, and use only 7 bits.
	for( uint8_t address = 8; address < 128; ++address ) {
		digitalWrite(LED_BUILTIN, address&8 ? HIGH : LOW);
		Wire.beginTransmission( address );
		// do we need to send at least one byte?
		if( Wire.endTransmission() == wire_result_ok ) {
			Serial.print( "Found open port at address " );
			print_hex_byte( address );
			Serial.println();
			++nr_found;
		}
	}

	digitalWrite(LED_BUILTIN, LOW);
	Serial.print("Scan complete, found ");
	Serial.print( nr_found, DEC );
	Serial.println(" open ports.");
}

int scan_for_newline( uint8_t* buffer, int len ) {
	for(int i = 0; i < len; i++) {
		if( buffer[i] == '\n' ) {
			return i;
		}
	}
	return len;
}

void to_upper( uint8_t* buffer, int len ) {
	for(int i = 0; i < len; i++) {
		if( buffer[i] >= 'a' && buffer[i] <= 'z' ) {
			buffer[i] -= 'a' - 'A';
		}
	}
}

// inplace conversion
int ascii_to_bin( uint8_t* buffer, int len ) {
	int index( 0 );
	for( int i = 0; i < len; i += 2 ) {
		buffer[index++] = ( ( buffer[i+0] - 'A' ) << 4 ) | ( ( buffer[i+1] - 'A' ) << 0 );
	}
	return index;
}

uint8_t compute_intel_hex_checksum( uint8_t* buffer, int len ) {
	uint8_t checksum( 0 );
	for( int i = 0; i < len; ++i ) {
		checksum += buffer[i];
	}
	return (~checksum) + 1;
}

int parse_intel_hex( uint8_t* buffer, int len, int& /*address*/, record_type& type ) throw() {
	if( buffer[0] != ':' ) {
		Serial.println( "Error: record must start with ':'" );
		return 0;
	}
	++buffer, --len; // strip ':'
	if( buffer[len-1] != '\n' ) {
		Serial.println( "Error: record must end with newline" );
		return 0;
	}
	--len; // strip newline
	for( int i = 0; i < len; ++ i ) {
		if( !( ( buffer[i] >= 'A' && buffer[i] <= 'F' ) || ( buffer[i] >= '0' && buffer[i] <= '9' ) ) ) {
			Serial.println( "Error: record contains non-hex data" );
			return 0;
		}
	}
	if( len % 2 != 0 ) {
		Serial.println( "Error: record corrupt" );
		return 0;
	}

	len = ascii_to_bin( buffer, len );

	// length + address + type + checksum
	if( buffer[0] + 1 + 2 + 1 + 1 != len ) {
		Serial.println( "Error: record length mismatch" );
		return 0;
	}

	uint8_t checksum = compute_intel_hex_checksum( buffer, len );
	if( buffer[len-1] != checksum ) {
		Serial.println( "Error: checksum mismatch" );
		return 0;
	}

	type = record_type( buffer[3] );
	if( type > record_type_end_of_file ) { // either data or EOF
		Serial.print( "Error: unsupported record type: " );
		print_hex_byte( buffer[3] );
		Serial.println();
		return 0;
	}

	// length, address, type and checksum are not part of data
	len -= 1 + 2 + 1 + 1;
	memmove( buffer, buffer + 3, len );

	return len;
}

void flash_data_from_serial() {
	Serial.println( "Flashing in progress..." );
	// Disable time out
	Serial.setTimeout(0);
	bool done( false );
	int index( 0 );
	while( !done ) {
		// Wait for data
		while( !Serial.available() );
		// Place what is available in buffer
		index += Serial.readBytes( (char*)buffer, BUFFER_LENGTH - index );

		// :nnaaaattdd..ddcc&
		// :00000001ff& = 12 bytes
		int newline = scan_for_newline( buffer, index );
		if( newline >= 12 && buffer[newline] == '\n' ) {
			to_upper( buffer, newline );
			int address(0);
			record_type type( record_type_data );
			int len = parse_intel_hex( buffer, newline, address, type );
			if( !len )
				return;
			switch( type ) {
				case record_type_end_of_file: {
					done = true;
					continue;
				}; break;
				case record_type_data: {
//					ddc_print( true ); // hmmm
					for( int i = 0; i < len; ++i ) {
						print_hex_byte( buffer[i] );
						Serial.print( ' ' );
					}
					Serial.println();
				}; break;
				default: {
					Serial.print("unhandled record type ");
					print_hex_byte(type);
					Serial.println();
				}
			}
		}
	}
	Serial.println( "Flash procedure complete!" );
}

void blink_status_led() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay( 100 );
	digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	//pinMode(TXLED0, OUTPUT);
	//pinMode(RXLED0, OUTPUT);
	Wire.begin();
	Serial.begin(9600);
	while(!Serial) {
		blink_status_led();
		delay( 100 );
	}
	Serial.println("NT68676 Flasher utility v0.1");
	Serial.println("--------------------------------------");
	Serial.print(" Buffer size: " );
	Serial.print( BUFFER_LENGTH, DEC );
	Serial.println( "bytes" );
}

void print_menu() {
	Serial.println(" 1) Scan i2c bus");
	Serial.print(" 2) Set target port (now: ");
	print_hex_byte( target_port );
	Serial.println( ")" );
	Serial.println(" 3) Read i2c port");
	Serial.println(" 4) Print data (human readable)");
	Serial.println(" 5) Print data as Intel HEX");
	Serial.println(" 6) Dump data from port (human readable)");
	Serial.println(" 7) Dump data from port as Intel hex");
	Serial.println(" 8) Write data from Intel HEX");
	Serial.println("> ");
}
void loop() {
	if( repaint_menu ) print_menu();

	// Always repaint, unless no key is pressed
	// (then this resets below)
	repaint_menu = true;

	// Timeout allows for blinking the led
	//	while(!Serial.available());
	Serial.setTimeout( 250 );
	switch(Serial.parseInt())	 {
		case 0: { // time out reached
			repaint_menu = false;
			blink_status_led();
		}; break;
		case 1: { // Scan i2c bus
			ddc_scan();
		}; break;
		case 2: { // set target port
			Serial.println( "port to scan?" );
			Serial.setTimeout( 5000 ); // 5sec
			byte address = Serial.parseInt();
			if( address > 7 ) {
				Serial.println( "Scanning port 0x" );
				Serial.print( address, HEX );
				Serial.println( ", please waiting..." );
				target_port = address;
			}
			else {
				Serial.println( "No valid address provided" );
			}
		}; break;
		case 3: { // read bytes from port
			ddc_read( target_port );
		}; break;
		case 4: { // print buffer
			ddc_print( 0, false );
		}; break;
		case 5: { // print buffer hex
			ddc_print( 0, true );
		}; break;
		case 6: { // dump data
			dump_all_data( false );
		}; break;
		case 7: { // dump data hex
			dump_all_data( true );
		}; break;
		case 8: { // upload
			//ut();
		}; break;
		default: {
			Serial.println("Error: Menu item does not exist.");
		}; break;
	}

	if( repaint_menu ) {
		// Some space before the next menu
		Serial.println("\n\n\n");
	}
}
