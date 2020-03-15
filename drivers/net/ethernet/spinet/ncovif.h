#ifndef _NCOVIF_H_
#define _NCOVIF_H_

#include "spinet.h"

#define ETH_ALEN        6           /* Octets in one ethernet addr                       */
#define ETH_HLEN        14          /* Total octets in header.                           */
#define ETH_ZLEN        60          /* Min. octets in frame sans FCS                     */
#define ETH_DATA_LEN    1500        /* Max. octets in payload                            */
#define ETH_FRAME_LEN   1514        /* Max. octets in frame sans FCS                     */
#define ETH_FCS_LEN	    4           /* Octets in the FCS                                 */

#define IP_HEADER_LEN   20

/* IP flags. */
#define IP_CE           0x8000      /* Flag: "Congestion"                                */
#define IP_DF           0x4000      /* Flag: "Don't Fragment"                            */
#define IP_MF           0x2000      /* Flag: "More Fragments"                            */
#define IP_OFFSET       0x1FFF      /* "Fragment Offset" part                            */

/* ARP protocol opcodes. */
#define	ARPOP_REQUEST   1           /* ARP request                                       */
#define	ARPOP_REPLY     2           /* ARP reply                                         */
#define	ARPOP_RREQUEST  3           /* RARP request                                      */
#define	ARPOP_RREPLY    4           /* RARP reply                                        */
#define	ARPOP_InREQUEST 8           /* InARP request                                     */
#define	ARPOP_InREPLY   9           /* InARP reply                                       */
#define	ARPOP_NAK       10          /* (ATM)ARP NAK                                      */

/* These are the defined Ethernet Protocol ID's */
#define ETH_P_LOOP      0x0060      /* Ethernet Loopback packet	                         */
#define ETH_P_PUP       0x0200      /* Xerox PUP packet                                  */
#define ETH_P_PUPAT     0x0201      /* Xerox PUP Addr Trans packet             	         */
#define ETH_P_IP        0x0800      /* Internet Protocol packet	                         */
#define ETH_P_X25       0x0805      /* CCITT X.25                                        */
#define ETH_P_ARP       0x0806      /* Address Resolution packet                         */
#define	ETH_P_BPQ       0x08FF      /* G8BPQ AX.25 Ethernet Packet
                                     [ NOT AN OFFICIALLY REGISTERED ID ]                 */
#define ETH_P_IEEEPUP   0x0a00      /* Xerox IEEE802.3 PUP packet                        */
#define ETH_P_IEEEPUPAT 0x0a01      /* Xerox IEEE802.3 PUP Addr Trans packet             */
#define ETH_P_DEC       0x6000      /* DEC Assigned proto                                */
#define ETH_P_DNA_DL    0x6001      /* DEC DNA Dump/Load                                 */
#define ETH_P_DNA_RC    0x6002      /* DEC DNA Remote Console                            */
#define ETH_P_DNA_RT    0x6003      /* DEC DNA Routing                                   */
#define ETH_P_LAT       0x6004      /* DEC LAT                                           */
#define ETH_P_DIAG      0x6005      /* DEC Diagnostics                                   */
#define ETH_P_CUST      0x6006      /* DEC Customer use                                  */
#define ETH_P_SCA       0x6007      /* DEC Systems Comms Arch                            */
#define ETH_P_TEB       0x6558      /* Trans Ether Bridging                              */
#define ETH_P_RARP      0x8035      /* Reverse Addr Res packet                           */
#define ETH_P_ATALK     0x809B      /* Appletalk DDP                                     */
#define ETH_P_AARP      0x80F3      /* Appletalk AARP                                    */
#define ETH_P_8021Q     0x8100      /* 802.1Q VLAN Extended Header                       */
#define ETH_P_IPX       0x8137      /* IPX over DIX                                      */
#define ETH_P_IPV6      0x86DD      /* IPv6 over bluebook                                */
#define ETH_P_PAUSE     0x8808      /* IEEE Pause frames. See 802.3 31B                  */
#define ETH_P_SLOW      0x8809      /* Slow Protocol. See 802.3ad 43B                    */
#define ETH_P_WCCP      0x883E      /* Web-cache coordination protocol
                                     * defined in draft-wilson-wrec-wccp-v2-00.txt       */
#define ETH_P_PPP_DISC  0x8863      /* PPPoE discovery messages                          */
#define ETH_P_PPP_SES   0x8864      /* PPPoE session messages                            */
#define ETH_P_MPLS_UC   0x8847      /* MPLS Unicast traffic                              */
#define ETH_P_MPLS_MC   0x8848      /* MPLS Multicast traffic                            */
#define ETH_P_ATMMPOA   0x884c      /* MultiProtocol Over ATM                            */
#define ETH_P_ATMFATE   0x8884      /* Frame-based ATM Transport over Ethernet           */
#define ETH_P_PAE       0x888E      /* Port Access Entity (IEEE 802.1X)                  */
#define ETH_P_AOE       0x88A2      /* ATA over Ethernet                                 */
#define ETH_P_TIPC      0x88CA      /* TIPC                                              */
#define ETH_P_1588      0x88F7      /* IEEE 1588 Timesync                                */
#define ETH_P_FCOE      0x8906      /* Fibre Channel over Ethernet                       */
#define ETH_P_FIP       0x8914      /* FCoE Initialization Protocol                      */
#define ETH_P_EDSA      0xDADA      /* Ethertype DSA [ NOT AN OFFICIALLY REGISTERED ID ] */

#define ICMP_ECHOREPLY      0       /* Echo Reply                                        */
#define ICMP_DEST_UNREACH   3       /* Destination Unreachable */
#define ICMP_SOURCE_QUENCH  4       /* Source Quench                                     */
#define ICMP_REDIRECT       5       /* Redirect (change route) */
#define ICMP_ECHO           8       /* Echo Request                                      */
#define ICMP_TIME_EXCEEDED  11      /* Time Exceeded                                     */
#define ICMP_PARAMETERPROB  12      /* Parameter Problem                                 */
#define ICMP_TIMESTAMP      13      /* Timestamp Request                                 */
#define ICMP_TIMESTAMPREPLY 14      /* Timestamp Reply                                   */
#define ICMP_INFO_REQUEST   15      /* Information Request                               */
#define ICMP_INFO_REPLY     16      /* Information Reply                                 */
#define ICMP_ADDRESS        17      /* Address Mask Request                              */
#define ICMP_ADDRESSREPLY   18      /* Address Mask Reply                                */
#define NR_ICMP_TYPES       18

#define __swap16(x)                      (((x>>8)|(x<<8))&0xFFFF)

#define ICMP_ECHO_GET_ID(icmph)          (icmph->un.echo.icmp_id)
#define ICMP_ECHO_GET_SEQ(icmph)         (icmph->un.echo.icmp_sequence)
#define ICMP_DEST_UNREACH_GET_MTU(icmph) (icmph->un.dest.nhop_mtu)

#define ICMP_ECHO_SET_ID(icmph, id)      (icmph->un.echo.icmp_id = id)
#define ICMP_ECHO_SET_SEQ(icmph, seq)    (icmph->un.echo.icmp_sequence = seq)

#define IP_NEXT_PTR(iph)                 ((uint8_t *)iph + (iph->ihl << 2))

struct arphdr {
	uint16_t		ar_hrd;             /* format of hardware address (BE)               */
	uint16_t		ar_pro;             /* format of protocol address (BE)               */
	uint8_t			ar_hln;             /* length of hardware address                    */
	uint8_t			ar_pln;             /* length of protocol address                    */
	uint16_t		ar_op;              /* ARP opcode (command)       (BE)               */
	uint8_t			ar_sha[ETH_ALEN];   /* sender hardware address                       */
	uint8_t			ar_sip[4];          /* sender IP address                             */
	uint8_t			ar_tha[ETH_ALEN];   /* target hardware address                       */
	uint8_t			ar_tip[4];          /* target IP address                             */
} __attribute__((packed, aligned(1)));

struct ethIIhdr {
	uint8_t			h_dest[ETH_ALEN];   /* destination eth addr                          */
	uint8_t			h_source[ETH_ALEN]; /* source ether addr                             */
	uint16_t		h_proto;            /* packet type ID field                          */
	uint8_t			data[ETH_DATA_LEN]; /* data fields                                   */
} __attribute__((packed, aligned(1)));

struct iphdr {
	uint8_t	 ihl:4,version:4;
	uint8_t	 tos;
	uint16_t tot_len;                   /* BE */
	uint16_t id;                        /* BE */
	uint16_t frag_off;                  /* BE */
	uint8_t	 ttl;
	uint8_t	 protocol;
	uint16_t check;
	uint32_t saddr;                     /* BE */
	uint32_t daddr;                     /* BE */
} __attribute__((packed, aligned(1)));

struct icmphdr {
	uint8_t	 icmp_type;
	uint8_t	 icmp_code;
	uint16_t icmp_checksum;
  union {
	struct {
		uint16_t icmp_id;               /* BE */
		uint16_t icmp_sequence;         /* BE */
	} echo;
	uint32_t gateway;                   /* BE */
	struct {
		uint16_t unused;                /* BE */
		uint16_t nhop_mtu;              /* BE */
	} dest;
	uint8_t	reserved[4];
  } un;
}__attribute__((packed, aligned(1)));

void eth_frame_process(struct spinet *priv, struct ethIIhdr *eth_frame);
void send_arp_response(struct spinet *priv);
void send_icmp_echo(struct spinet *priv);

#endif