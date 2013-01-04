/**
 * @file
 * @brief Simple scanf implementation.
 *
 * @date 10.02.09
 * @author Alexandr Batyukov
 *          - Initial implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdbool.h>

/*TODO: throw out.*/
/**
 * convert digit character to integer
 * @param digit character for converting
 * @param base for converting
 * @return converted symbol
 * @return -1 if error
 */
int ch_to_digit(char ch, int base) {
        ch = toupper(ch);
        switch (base) {
        case 16: {
                if (ch >= '0' && ch <= '9') {
                        return (ch - '0');
                } else if (ch >= 'A' && ch <= 'F') {
                        return (ch - 'A' + 0x0A);
                }
                return -1;
        }
        case 10: {
                if (ch >= '0' && ch <= '9') {
                        return (ch - '0');
                }
                return -1;
        }
        case 8: {
                if (ch >= '0' && ch <= '7') {
                        return (ch - '0');
                }
                return -1;
        }
        default:
                return -1;
        }
        return -1;
}

extern int ungetchar(int ch);

static void unscanchar(char **str, int ch) {
        if (str) {

                *str --;
                **str = ch;
#if 0
                /*int *p;
                 p = *str - 4;
                 *p = ch;
                 *str = p;*/
#endif
        } else 
               return;
}

static int scanchar(char **str) {
        int ch;
        if (str) {
                ch = **str;
                (*str)++;
                return ch;

        } else 
                return EOF;
}

static bool is_space(int ch) {
        if ((ch == ' ') || (ch == '\t') || (ch == '\n'))
                return true;
        return false;
}

static int trim_leading(char **str) {
        int ch;

        do {
                ch = scanchar(str);
                /*when break*/
                if (ch == EOF)
                        break;
        } while (is_space(ch));

        unscanchar(str, ch);
        return ch;
}

static int scan_int(char **in, int base, int width) {
        int neg = 0;
        int dst = 0;
        int ch;
        int i;

        if (EOF == (ch = trim_leading(in)))
                return 0;/*error*/

        if ((ch == '-') || (ch == '+')) {
                neg = (ch == '-');
        } else {
                dst = ch_to_digit(ch, base);
        }

        for (i = 0; (ch = (int) toupper(scanchar(in))) != EOF; i++) {
                if (!(base == 10 ? isdigit(ch) : isxdigit(ch)) || (0 == width)) {
                        unscanchar(in, ch);
                        /*end conversion*/
                        break;
                }
                unscanchar(in, ch);

                dst = base * dst + ch_to_digit(ch, base);
        }

        if (neg)
                dst = -dst;
        return dst;
}


static double scan_double(char **in, int base, int width) {
        int neg = 0, found_decimal = 0;
        double dst = 0, part = 0, upart=0;
        int ch;
        int i;

        if (EOF == (ch = trim_leading(in)))
                return 0;/*error*/

        if ((ch == '-') || (ch == '+')) {
                neg = (ch == '-');
        } else {
                dst = ch_to_digit(ch, base);
        }

        for (i = 0; (ch = scanchar(in)) != EOF; i++) {
                if (!(base == 10 ? isdigit(ch) : isxdigit(ch)) || (0 == width)) {
			if(ch!='.') {
                        	unscanchar(in,ch);
                        	break;
			}
			else {
				upart = dst;
				dst=1;
				found_decimal=1;
				continue;
			}
                }
		if(found_decimal) {
			dst /= base;
                	/*for different bases*/
                	if (base >10)
                		part += (ch - '0' - 7)*dst;
                	else
                		part += (ch - '0')*dst;
		}
		else {
			dst *= base;
                	/*for different bases*/
                	if (base >10)
                		dst += (ch - '0' - 7);
                	else
                		dst += (ch - '0');
		}
        }
	if(found_decimal)
		dst=part+upart;
        if (neg)
       		dst = -dst;
        return dst;
}


static int scan(char **in, const char *fmt, va_list args) {
        int width;
        int converted = 0;

        while (*fmt != '\0') {
                if (*fmt == '%') {
                        fmt++;
                        width = 80;

                        if (*fmt == '\0')
                                break;

                        if (isdigit((int) *fmt))
                                width = 0;

                        while (isdigit((int) *fmt)) {

                                width = width * 10 + (*fmt++ - '0');
                        }

                        switch (*fmt) {
                        case 's': {
                                char *dst = va_arg(args, char*);
                                int ch;
#if 0
                                trim_leading(in);
#endif
                                while (EOF != (ch = scanchar(in)) && width--)
                                        *dst++ = (char) ch;
                                *dst = '\0';

                                ++converted;
                        }
                                continue;
                        case 'c': {
                                int dst;

#if 0
                                trim_leading(in);
#endif
                                dst = scanchar(in);
                                *va_arg(args, char*) = dst;
                                ++converted;

                        }
                                continue;
                        case 'u':
                        case 'd': {
                                int dst;
                                dst = scan_int(in, 10, width);

                                *va_arg(args, int*) = dst;

                                ++converted;
                        }
                                continue;

                        case 'D': {
                                        double dst;
                                        dst = scan_double(in,10,width);
                                        *va_arg(args, double*) = dst;
                                        ++converted;
                                }
                                continue;

                        case 'o': {
                                int dst;
                                dst = scan_int(in, 8, width);

                                *va_arg(args, int*) = dst;

                                ++converted;
                        }
                                continue;
#if 0
                        case 'O': {
                                double dst;
                                dst = scan_double(in,8,widht);
                                va_arg(args, int) = dst;
                                ++converted;
                        }
                                continue;
#endif
                        case 'x': {
                                int dst;
                                dst = scan_int(in, 16, width);
                                *va_arg(args, int*) = dst;

                                ++converted;
                        }
                                continue;
#if 0
                        case 'X': {
                                double dst;
                                dst = scan_double(in, 16, widht);
                                va_arg(args, int) = dst;
                                ++converted;
                        }
                                continue;
#endif
                        case 'f': {
                                /*TODO: scanf haven't realized float variable operations*/
                        }
                                continue;
                        }
                } else {
                        if (*fmt++ != *(*in)++) {
                                return converted;
                        }
                }
        }

        return converted;
}

int scanf(const char *format, ...) {
        va_list args;
        int rv;

        va_start(args, format);
        rv = scan(0, format, args);
        va_end(args);

        return rv;
}

int sscanf(const char *out, const char *format, ...) {
        va_list args;
        int rv;

        va_start(args, format);
        rv = scan((char **)&out, format, args);
        va_end (args);

        return rv;
}


