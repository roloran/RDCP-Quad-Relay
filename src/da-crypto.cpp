#include "da-crypto.h"
#include "serial.h"
#include "lora.h"
#include "rdcp-common.h"

SchnorrSigCtx ssc;
bool ssc_initialized = false;

extern da_config CFG; 

bool encrypt_aes256gcm(uint8_t *plaintext, size_t ptsize, uint8_t *adata, size_t adatasize, uint8_t *key, size_t keysize, uint8_t *iv, size_t ivsize, uint8_t *out_ciphertext, uint8_t *out_tag, size_t tagsize) 
{
  GCM<AES256> gcm;
  gcm.setKey(key, keysize);
  gcm.setIV(iv, ivsize);
  gcm.addAuthData(adata, adatasize);
  gcm.encrypt(out_ciphertext, plaintext, ptsize);
  gcm.computeTag(out_tag, tagsize);
  return true;
}

bool decrypt_aes256gcm(uint8_t *ciphertext, size_t csize, uint8_t *adata, size_t adatasize, uint8_t *key, size_t keysize, uint8_t *iv, size_t ivsize, uint8_t *tag, size_t tagsize, uint8_t *out_plaintext) 
{
  GCM<AES256> gcm;
  gcm.setKey(key, keysize);
  gcm.setIV(iv, ivsize);
  gcm.addAuthData(adata, adatasize);
  gcm.decrypt(out_plaintext, ciphertext, csize);
  if (!gcm.checkTag(tag, tagsize)) return false;
  return true;
}

bool schnorr_init_ctx(void)
{
  if (ssc_initialized) return true;

  size_t seed_len = MBEDTLS_CTR_DRBG_MAX_SEED_INPUT - MBEDTLS_CTR_DRBG_ENTROPY_LEN;
  uint8_t seed[seed_len];    
  esp_fill_random(seed, seed_len);

  ssc = SchnorrSigCtx();
  int res = ssc.init(seed, seed_len);
  if (res != RESULT_OK)
  {
    serial_writeln("ERROR: schnorr_init_ctx() failed");
    return false;
  }

  ssc_initialized = true;
  return true;
}

int schnorr_create_signature(uint8_t *data, uint8_t datalen, uint8_t *targetbuffer)
{
  schnorr_init_ctx();

  SchnorrSigSign sss = SchnorrSigSign();
  int res = sss.init(&ssc, CFG.myprivkey);
  if (res != RESULT_OK)
  {
    char buf[2*INFOLEN];
    snprintf(buf, 2*INFOLEN, "ERROR: schnorr_create_signature() could not initialize with private key %s", CFG.myprivkey);
    serial_writeln(buf);
    return RESULT_FAIL;
  }

  struct SchnorrSigCtx::signature new_sig;
  res = sss.sign((const unsigned char*)data, datalen, &new_sig);
  if (res != RESULT_OK)
  {
    serial_writeln("ERROR: schnorr_create_signature() signing failed");
    return RESULT_FAIL;
  }

  int tbi = 0;
  for (int i=0; i < new_sig.point_len; i++) targetbuffer[tbi++] = new_sig.point[i];
  for (int i=0; i < new_sig.sig_len  ; i++) targetbuffer[tbi++] = new_sig.sig[i];

  return tbi;
}

bool schnorr_verify_signature(uint8_t *data, uint8_t datalen, uint8_t *signature)
{
  schnorr_init_ctx();

  SchnorrSigVerify ssv = SchnorrSigVerify();
  int res = ssv.init(&ssc, CFG.hqpubkey);
                               
  if (res != RESULT_OK)
  {
    char msg[INFOLEN];
    snprintf(msg, INFOLEN, "ERROR: schnorr_verify_signature() could not initialize (res %d) with HQ public key %s", res, CFG.hqpubkey);
    serial_writeln(msg);
    return false;
  }

  struct SchnorrSigCtx::signature sig;
  sig.point_len = POINT_LEN;
  for (int i=0; i<POINT_LEN; i++) sig.point[i] = signature[i];
  sig.sig_len = SIG_LEN;
  for (int i=0; i<SIG_LEN; i++) sig.sig[i] = signature[i+POINT_LEN];

  res = ssv.verify((const unsigned char*)data, datalen, &sig);
  if (res == RESULT_OK) return true;

  return false;
}

/* EOF */