#ifndef TWEETNACL_H
#define TWEETNACL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define crypto_sign_BYTES 64
#define crypto_sign_PUBLICKEYBYTES 32
#define crypto_sign_SECRETKEYBYTES 64

/**
 * Generate an Ed25519 keypair
 * @param pk Public key output (32 bytes)
 * @param sk Secret key output (64 bytes)
 * @return 0 on success
 */
int crypto_sign_keypair(uint8_t *pk, uint8_t *sk);

/**
 * Sign a message using Ed25519
 * @param sm Signed message output (message + 64 byte signature prepended)
 * @param smlen Length of signed message output
 * @param m Message to sign
 * @param mlen Length of message
 * @param sk Secret key (64 bytes)
 * @return 0 on success
 */
int crypto_sign(uint8_t *sm, uint64_t *smlen,
                const uint8_t *m, uint64_t mlen,
                const uint8_t *sk);

/**
 * Verify and open a signed message
 * @param m Message output
 * @param mlen Length of message output
 * @param sm Signed message
 * @param smlen Length of signed message
 * @param pk Public key (32 bytes)
 * @return 0 on success, -1 on verification failure
 */
int crypto_sign_open(uint8_t *m, uint64_t *mlen,
                     const uint8_t *sm, uint64_t smlen,
                     const uint8_t *pk);

/**
 * Create a detached signature
 * @param sig Signature output (64 bytes)
 * @param siglen Length of signature output (will be 64)
 * @param m Message to sign
 * @param mlen Length of message
 * @param sk Secret key (64 bytes)
 * @return 0 on success
 */
int crypto_sign_detached(uint8_t *sig, uint64_t *siglen,
                         const uint8_t *m, uint64_t mlen,
                         const uint8_t *sk);

/**
 * Verify a detached signature
 * @param sig Signature (64 bytes)
 * @param m Message
 * @param mlen Length of message
 * @param pk Public key (32 bytes)
 * @return 0 on success, -1 on verification failure
 */
int crypto_sign_verify_detached(const uint8_t *sig,
                                const uint8_t *m, uint64_t mlen,
                                const uint8_t *pk);

/**
 * Generate random bytes using ESP32 hardware RNG
 * @param buf Buffer to fill
 * @param len Number of bytes to generate
 */
void randombytes(uint8_t *buf, uint64_t len);

#ifdef __cplusplus
}
#endif

#endif /* TWEETNACL_H */
