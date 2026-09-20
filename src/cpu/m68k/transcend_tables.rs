// Tables for `transcend.rs`, generated rather than transcribed, and included
// into it rather than compiled as a module of their own so they share its
// `Wide`.
//
// Both are computed from exact rational arithmetic at two hundred decimal
// digits and rounded once, ties to even; the commit that added them records
// the program. Neither is anybody's code: they are the binary expansions of
// `1/n` and of `2/π`.

/// `1/n`, indexed by `n`, for the divisors the series in `transcend.rs` use.
static RECIPROCAL: [Wide; 90] = [
    // 1/0 is never used; the table is indexed by the divisor.
    Wide::ZERO,
    Wide { sign: false, exp: 0, frac: 0x80000000000000000000000000000000 }, // 1/1
    Wide { sign: false, exp: -1, frac: 0x80000000000000000000000000000000 }, // 1/2
    Wide { sign: false, exp: -2, frac: 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab }, // 1/3
    Wide { sign: false, exp: -2, frac: 0x80000000000000000000000000000000 }, // 1/4
    Wide { sign: false, exp: -3, frac: 0xcccccccccccccccccccccccccccccccd }, // 1/5
    Wide { sign: false, exp: -3, frac: 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab }, // 1/6
    Wide { sign: false, exp: -3, frac: 0x92492492492492492492492492492492 }, // 1/7
    Wide { sign: false, exp: -3, frac: 0x80000000000000000000000000000000 }, // 1/8
    Wide { sign: false, exp: -4, frac: 0xe38e38e38e38e38e38e38e38e38e38e4 }, // 1/9
    Wide { sign: false, exp: -4, frac: 0xcccccccccccccccccccccccccccccccd }, // 1/10
    Wide { sign: false, exp: -4, frac: 0xba2e8ba2e8ba2e8ba2e8ba2e8ba2e8ba }, // 1/11
    Wide { sign: false, exp: -4, frac: 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab }, // 1/12
    Wide { sign: false, exp: -4, frac: 0x9d89d89d89d89d89d89d89d89d89d89e }, // 1/13
    Wide { sign: false, exp: -4, frac: 0x92492492492492492492492492492492 }, // 1/14
    Wide { sign: false, exp: -4, frac: 0x88888888888888888888888888888889 }, // 1/15
    Wide { sign: false, exp: -4, frac: 0x80000000000000000000000000000000 }, // 1/16
    Wide { sign: false, exp: -5, frac: 0xf0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f1 }, // 1/17
    Wide { sign: false, exp: -5, frac: 0xe38e38e38e38e38e38e38e38e38e38e4 }, // 1/18
    Wide { sign: false, exp: -5, frac: 0xd79435e50d79435e50d79435e50d7943 }, // 1/19
    Wide { sign: false, exp: -5, frac: 0xcccccccccccccccccccccccccccccccd }, // 1/20
    Wide { sign: false, exp: -5, frac: 0xc30c30c30c30c30c30c30c30c30c30c3 }, // 1/21
    Wide { sign: false, exp: -5, frac: 0xba2e8ba2e8ba2e8ba2e8ba2e8ba2e8ba }, // 1/22
    Wide { sign: false, exp: -5, frac: 0xb21642c8590b21642c8590b21642c859 }, // 1/23
    Wide { sign: false, exp: -5, frac: 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab }, // 1/24
    Wide { sign: false, exp: -5, frac: 0xa3d70a3d70a3d70a3d70a3d70a3d70a4 }, // 1/25
    Wide { sign: false, exp: -5, frac: 0x9d89d89d89d89d89d89d89d89d89d89e }, // 1/26
    Wide { sign: false, exp: -5, frac: 0x97b425ed097b425ed097b425ed097b42 }, // 1/27
    Wide { sign: false, exp: -5, frac: 0x92492492492492492492492492492492 }, // 1/28
    Wide { sign: false, exp: -5, frac: 0x8d3dcb08d3dcb08d3dcb08d3dcb08d3e }, // 1/29
    Wide { sign: false, exp: -5, frac: 0x88888888888888888888888888888889 }, // 1/30
    Wide { sign: false, exp: -5, frac: 0x84210842108421084210842108421084 }, // 1/31
    Wide { sign: false, exp: -5, frac: 0x80000000000000000000000000000000 }, // 1/32
    Wide { sign: false, exp: -6, frac: 0xf83e0f83e0f83e0f83e0f83e0f83e0f8 }, // 1/33
    Wide { sign: false, exp: -6, frac: 0xf0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f1 }, // 1/34
    Wide { sign: false, exp: -6, frac: 0xea0ea0ea0ea0ea0ea0ea0ea0ea0ea0ea }, // 1/35
    Wide { sign: false, exp: -6, frac: 0xe38e38e38e38e38e38e38e38e38e38e4 }, // 1/36
    Wide { sign: false, exp: -6, frac: 0xdd67c8a60dd67c8a60dd67c8a60dd67d }, // 1/37
    Wide { sign: false, exp: -6, frac: 0xd79435e50d79435e50d79435e50d7943 }, // 1/38
    Wide { sign: false, exp: -6, frac: 0xd20d20d20d20d20d20d20d20d20d20d2 }, // 1/39
    Wide { sign: false, exp: -6, frac: 0xcccccccccccccccccccccccccccccccd }, // 1/40
    Wide { sign: false, exp: -6, frac: 0xc7ce0c7ce0c7ce0c7ce0c7ce0c7ce0c8 }, // 1/41
    Wide { sign: false, exp: -6, frac: 0xc30c30c30c30c30c30c30c30c30c30c3 }, // 1/42
    Wide { sign: false, exp: -6, frac: 0xbe82fa0be82fa0be82fa0be82fa0be83 }, // 1/43
    Wide { sign: false, exp: -6, frac: 0xba2e8ba2e8ba2e8ba2e8ba2e8ba2e8ba }, // 1/44
    Wide { sign: false, exp: -6, frac: 0xb60b60b60b60b60b60b60b60b60b60b6 }, // 1/45
    Wide { sign: false, exp: -6, frac: 0xb21642c8590b21642c8590b21642c859 }, // 1/46
    Wide { sign: false, exp: -6, frac: 0xae4c415c9882b9310572620ae4c415ca }, // 1/47
    Wide { sign: false, exp: -6, frac: 0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab }, // 1/48
    Wide { sign: false, exp: -6, frac: 0xa72f05397829cbc14e5e0a72f0539783 }, // 1/49
    Wide { sign: false, exp: -6, frac: 0xa3d70a3d70a3d70a3d70a3d70a3d70a4 }, // 1/50
    Wide { sign: false, exp: -6, frac: 0xa0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a1 }, // 1/51
    Wide { sign: false, exp: -6, frac: 0x9d89d89d89d89d89d89d89d89d89d89e }, // 1/52
    Wide { sign: false, exp: -6, frac: 0x9a90e7d95bc609a90e7d95bc609a90e8 }, // 1/53
    Wide { sign: false, exp: -6, frac: 0x97b425ed097b425ed097b425ed097b42 }, // 1/54
    Wide { sign: false, exp: -6, frac: 0x94f2094f2094f2094f2094f2094f2095 }, // 1/55
    Wide { sign: false, exp: -6, frac: 0x92492492492492492492492492492492 }, // 1/56
    Wide { sign: false, exp: -6, frac: 0x8fb823ee08fb823ee08fb823ee08fb82 }, // 1/57
    Wide { sign: false, exp: -6, frac: 0x8d3dcb08d3dcb08d3dcb08d3dcb08d3e }, // 1/58
    Wide { sign: false, exp: -6, frac: 0x8ad8f2fba9386822b63cbeea4e1a08ae }, // 1/59
    Wide { sign: false, exp: -6, frac: 0x88888888888888888888888888888889 }, // 1/60
    Wide { sign: false, exp: -6, frac: 0x864b8a7de6d1d60864b8a7de6d1d6086 }, // 1/61
    Wide { sign: false, exp: -6, frac: 0x84210842108421084210842108421084 }, // 1/62
    Wide { sign: false, exp: -6, frac: 0x82082082082082082082082082082082 }, // 1/63
    Wide { sign: false, exp: -6, frac: 0x80000000000000000000000000000000 }, // 1/64
    Wide { sign: false, exp: -7, frac: 0xfc0fc0fc0fc0fc0fc0fc0fc0fc0fc0fc }, // 1/65
    Wide { sign: false, exp: -7, frac: 0xf83e0f83e0f83e0f83e0f83e0f83e0f8 }, // 1/66
    Wide { sign: false, exp: -7, frac: 0xf4898d5f85bb39503d226357e16ece54 }, // 1/67
    Wide { sign: false, exp: -7, frac: 0xf0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f1 }, // 1/68
    Wide { sign: false, exp: -7, frac: 0xed7303b5cc0ed7303b5cc0ed7303b5cc }, // 1/69
    Wide { sign: false, exp: -7, frac: 0xea0ea0ea0ea0ea0ea0ea0ea0ea0ea0ea }, // 1/70
    Wide { sign: false, exp: -7, frac: 0xe6c2b4481cd85689039b0ad12073615a }, // 1/71
    Wide { sign: false, exp: -7, frac: 0xe38e38e38e38e38e38e38e38e38e38e4 }, // 1/72
    Wide { sign: false, exp: -7, frac: 0xe070381c0e070381c0e070381c0e0704 }, // 1/73
    Wide { sign: false, exp: -7, frac: 0xdd67c8a60dd67c8a60dd67c8a60dd67d }, // 1/74
    Wide { sign: false, exp: -7, frac: 0xda740da740da740da740da740da740da }, // 1/75
    Wide { sign: false, exp: -7, frac: 0xd79435e50d79435e50d79435e50d7943 }, // 1/76
    Wide { sign: false, exp: -7, frac: 0xd4c77b03531dec0d4c77b03531dec0d5 }, // 1/77
    Wide { sign: false, exp: -7, frac: 0xd20d20d20d20d20d20d20d20d20d20d2 }, // 1/78
    Wide { sign: false, exp: -7, frac: 0xcf6474a8819ec8e951033d91d2a2067b }, // 1/79
    Wide { sign: false, exp: -7, frac: 0xcccccccccccccccccccccccccccccccd }, // 1/80
    Wide { sign: false, exp: -7, frac: 0xca4587e6b74f0329161f9add3c0ca458 }, // 1/81
    Wide { sign: false, exp: -7, frac: 0xc7ce0c7ce0c7ce0c7ce0c7ce0c7ce0c8 }, // 1/82
    Wide { sign: false, exp: -7, frac: 0xc565c87b5f9d4d1bc2503159721ed7e7 }, // 1/83
    Wide { sign: false, exp: -7, frac: 0xc30c30c30c30c30c30c30c30c30c30c3 }, // 1/84
    Wide { sign: false, exp: -7, frac: 0xc0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c1 }, // 1/85
    Wide { sign: false, exp: -7, frac: 0xbe82fa0be82fa0be82fa0be82fa0be83 }, // 1/86
    Wide { sign: false, exp: -7, frac: 0xbc52640bc52640bc52640bc52640bc52 }, // 1/87
    Wide { sign: false, exp: -7, frac: 0xba2e8ba2e8ba2e8ba2e8ba2e8ba2e8ba }, // 1/88
    Wide { sign: false, exp: -7, frac: 0xb81702e05c0b81702e05c0b81702e05c }, // 1/89
];

/// The fractional bits of `2/π`, most significant first: word 0 holds bits
/// 1 to 64 of `0.101000101111100…`.
///
/// Sixteen thousand six hundred and forty of them, which is enough to reduce
/// **every** representable extended-precision argument exactly: the largest
/// has an exponent of 16383, and Payne–Hanek needs the bits from just above
/// that binade down for another sixty-four (the significand) plus the
/// hundred and twenty-eight this module carries.
static TWO_OVER_PI: [u64; 260] = [
    0xa2f9836e4e441529, 0xfc2757d1f534ddc0, 0xdb6295993c439041, 0xfe5163abdebbc561,
    0xb7246e3a424dd2e0, 0x06492eea09d1921c, 0xfe1deb1cb129a73e, 0xe88235f52ebb4484,
    0xe99c7026b45f7e41, 0x3991d639835339f4, 0x9c845f8bbdf9283b, 0x1ff897ffde05980f,
    0xef2f118b5a0a6d1f, 0x6d367ecf27cb09b7, 0x4f463f669e5fea2d, 0x7527bac7ebe5f17b,
    0x3d0739f78a5292ea, 0x6bfb5fb11f8d5d08, 0x56033046fc7b6bab, 0xf0cfbc209af4361d,
    0xa9e391615ee61b08, 0x6599855f14a06840, 0x8dffd8804d732731, 0x06061556ca73a8c9,
    0x60e27bc08c6b47c4, 0x19c367cddce8092a, 0x8359c4768b961ca6, 0xddaf44d15719053e,
    0xa5ff07053f7e33e8, 0x32c2de4f98327dbb, 0xc33d26ef6b1e5ef8, 0x9f3a1f35caf27f1d,
    0x87f121907c7c246a, 0xfa6ed5772d30433b, 0x15c614b59d19c3c2, 0xc4ad414d2c5d000c,
    0x467d862d71e39ac6, 0x9b0062337cd2b497, 0xa7b4d55537f63ed7, 0x1810a3fc764d2a9d,
    0x64abd770f87c6357, 0xb07ae715175649c0, 0xd9d63b3884a7cb23, 0x24778ad623545ab9,
    0x1f001b0af1dfce19, 0xff319f6a1e666157, 0x9947fbacd87f7eb7, 0x652289e83260bfe6,
    0xcdc4ef09366cd43f, 0x5dd7de16de3b5892, 0x9bde2822d2e88628, 0x4d58e232cac61ba5,
    0x77078b5987685af3, 0xd216bbfe12735468, 0x5381ac8baee9318a, 0x3418c7ee6d6aca11,
    0x1058bfb20ae9313e, 0x62d9c66a55e466f2, 0x4fcdb4d8409d15c5, 0x0800b80ac9499d3c,
    0x5b2df10c9285e0e6, 0x8360315f11ba79d8, 0xb52d29106b7639a7, 0x3a810ca7103a07f5,
    0xcadcd29bf57993b2, 0x6bf3f495ecc7f1b5, 0x5f2b903142373cbe, 0x5861f506dd5ebe11,
    0xd81090367477b4ea, 0x2d06fd9d5f1e18f7, 0x6c6b40c40e7a1752, 0xe84f6bedbbe681c6,
    0x4722d1fedc29650a, 0x3d8c16ca64bf6f46, 0xed7f452428dfb1f2, 0xa36fe6c7d1c02088,
    0x9bb812479eb1afcd, 0x0baf1a381c4f2f9c, 0x7c42799d40e16d32, 0x9da1297198b0be54,
    0x6457cec3f3ce56e1, 0x1ccddfb67f99c9da, 0x2e164a167dc64004, 0x19ec1077854257dc,
    0x74978feff1111e15, 0x119393b289782574, 0x0598f8f50080af91, 0x9f6edfed580ebed3,
    0x5cad9f3f848063b7, 0x7d22bd7b4d0e3745, 0xb7473f929e88f0f6, 0x5f1445f25c63df90,
    0x60cb1d3186c0459c, 0x9dfad0a4a483b374, 0x0ce283bd80157a61, 0xed244527d70e6493,
    0x3105d0e4ff1cd1cc, 0x4cee55b4b37e7295, 0x5900129f71f137b6, 0xc764fb325a9b7970,
    0x99052342c1622fcd, 0x588e5d898fb2972d, 0xa7a06e3231545bd5, 0x8b6d22c1c9a2dbc9,
    0x5fbb3259b57af7b6, 0x7d2b7389afa58b32, 0x40ea414382b2b582, 0xd5dfaf53377445e1,
    0x3c6a648572a38e46, 0xbc3b6ececa366f3f, 0x0950228f3fd53865, 0x64afcad43e17ebe1,
    0x904ac40a71d9060f, 0x0e0546c294dccb27, 0x09b4d3bdc711ae6d, 0x1b0a068cb35bd926,
    0xff12e6476d3a5099, 0x146aab85a5404c2b, 0x905058a0aa9b459f, 0x790a835aa3994d8c,
    0xbb107bf6880e3dfa, 0xec784328719a592c, 0x5d1f7963dc870cb1, 0xab4aef4307887376,
    0xa950af8e3c4a2383, 0x9b8f4d21d962ac2b, 0xb4c04cfb835b8e38, 0xbaea98c74f100fee,
    0x72fe2fbe3964d361, 0x493cb23d58c5fc43, 0xd3652990e11d1fc5, 0x964884b0e47ca2e7,
    0xa143736e73492662, 0xae765fcbd19f85cf, 0x4bb3fceac7809f13, 0x8946c9dcaa6718d3,
    0x0a765e87ae1586a4, 0x3626bb8856779e9d, 0x8aae2155a51bb6a3, 0x5fd6980c7af29f11,
    0x15cba462c10ba341, 0x72f374c43f9cb493, 0x03e44a81a57e7551, 0x731ea12c84bcc0d7,
    0x2c18e74e7416061c, 0x8c4c971a7ad17ea3, 0x2d3bc7c34318db84, 0x90143a4432b0bc87,
    0x441781d2ac452724, 0x1376f052a75c0367, 0x35c0f6c534c30698, 0x8d7d7ccc43740c40,
    0xd9ae3a4028e3242c, 0xa2b5c5a00248c7b7, 0xd7931717624d204e, 0x058a385f73427c2e,
    0x17ae36ce75825d4c, 0x1c26f6e1677b3ee5, 0xb4fbcd49181c3d8a, 0xfcbdff82796ae25f,
    0xf6abf38bff567d24, 0x3e10156c8f1547e5, 0x7ee7fec05292533f, 0x535e3b19336af381,
    0x342b684f0462df7f, 0x2eb3be982f2056eb, 0x8f56b233a5aa0f02, 0x2a6b63807d049867,
    0xe3515726ec38eab4, 0xae8590af4574f478, 0x98db42e8abc81bc1, 0xd53b4e726c8524c3,
    0x50a7c8ecca5e30fd, 0xeed17bd7f687289c, 0xbf26074e21940c8b, 0x16c5600f53e192ed,
    0x82fed859a2b9cc5c, 0x2571adeb67907f43, 0x98488747c65daaf6, 0xe224d8726ad6ef32,
    0xcec2770cde3f85f4, 0x892b558000334f79, 0xe9cc78f9b207a8f1, 0xd5887ab659d93803,
    0x002f766353cf3487, 0x6a9e3ef0aa2facea, 0x5a5f5abb24b76438, 0xfda415eff70cecdd,
    0x2b13f70fba5522cb, 0x68a2206a872df64d, 0xf2a805940d351c4a, 0xf8ba68f6bae2113e,
    0x850e336169f6f513, 0x009bff2824235659, 0xab56cb06e9fc1c45, 0x599ff8dfdb9070d6,
    0x69540e7f83a4f9b3, 0xbfaf94d8218ed5e7, 0xbdcb0b0730d388bd, 0x265edad6af6e9c5c,
    0x7a0e8c16747ae7f9, 0xee272c996a086f14, 0x59eddb5f0c53f75e, 0x57a82b528e128a5c,
    0x8772ca8c4c9dd59a, 0x44ec593d3ed32793, 0x3edd58b867f3a7e1, 0xcedeeed76300c582,
    0x17972876ad3102ad, 0x36f0297f302765a7, 0xd2cf153b844c5c0c, 0x33f8a4011be1af0e,
    0x32c83de70388509a, 0xaf2220d52302d914, 0x1a16238ba62846fb, 0xbcd1a4e0810ee146,
    0xd795cf85539f35a9, 0x8ecea0e5d5e4126f, 0xde5a8ef8a592e17c, 0xf3b18cbafe236f04,
    0x0a2bd1d3706cab74, 0x9abc5daf9bca5abd, 0xa4238b23a432a6f8, 0xb44cef96d01377de,
    0xfc8fda75bdd9a345, 0x47e9d9539f20950f, 0xea8b6b2ee6dee9aa, 0xe5c84b87ffc428b0,
    0xdc32017207919be3, 0x53e320ac3a58968a, 0xfc6bd49a76b10627, 0x843a0bf6b635cd7a,
    0xa8fd880455d0526f, 0x42241d3bb1af9362, 0x2e5bbcc826c906c5, 0x36cffea8d75306e7,
    0x4b03055e8906f646, 0x705210943e036d2e, 0x163113e575e7cdc4, 0x92b0986abe54b864,
    0x5f3c98a6f984124a, 0x47231133da488a5c, 0x520e9c8a54d5e406, 0xdb38796ff16baec0,
    0xed02791bc1136371, 0x350d9def99cf9224, 0x6e1b7bd0dcd48838, 0x0d8c9a0782d5f36c,
    0x06a5ef94fc862b65, 0x1262749e97ab9031, 0x60d3f090a3b42d35, 0x2961374f4dbb39a0,
    0x6d46f477f701f2ca, 0x2d1c96be1ebb206a, 0x05997db94a4e6d17, 0x58dc83fd4b1b2452,
    0x5fbcadc91e2147cb, 0x8ba33991acfdc941, 0x6ef5df38c6fcffbf, 0xa27626bbdac2c2b5,
];
