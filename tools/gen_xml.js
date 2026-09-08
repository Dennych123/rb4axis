#!/usr/bin/env node
// sim/*.st + sim/*.tsv  ->  satu berkas XML yang di-import Sysmac Studio.
//
//   node blurobot/tools/gen_xml.js            tulis sim/BlurobotSim.xml
//   node blurobot/tools/gen_xml.js --check    cuma periksa, exit 1 kalau sudah basi
//
// Gunanya menghapus langkah paling rawan di SETUP.md: menempel dua FB, tabel
// variabel FB, tabel global, tabel program dan badan ST satu per satu. Tiap
// tempelan yang kolomnya melenceng diterima Studio tanpa keluhan, dan yang salah
// baru kelihatan waktu Build - atau lebih buruk, waktu lengannya bergerak.
//
// BENTUKNYA DITIRU DARI Sample.xml MILIK OMRON, bukan dikarang:
//
//   <BodyContent xsi:type="ST"><ST>teks ST polos</ST></BodyContent>
//   <Variable name="x" orderWithinParamSet="0"><Type><TypeName>BOOL</TypeName></Type>
//
// `orderWithinParamSet` itu SATU pencacah yang jalan terus dari InputVars ke
// OutputVars (di Sample: input 0, output 1), bukan dua pencacah terpisah. Angka itu
// yang menentukan urutan pin kotaknya di ladder.
//
// Akhiran baris di dalam <ST> ditulis LF dan itu BUKAN kelalaian: pembaca XML apa
// pun menormalkan CRLF jadi LF sebelum teksnya sampai ke Studio (XML 1.0 bagian
// 2.11), jadi memaksa CRLF di sini tidak ada gunanya. Aturan "ST wajib CRLF" itu
// milik jalur .smc2 (scripts/smc2_section.js) yang menulis ke dalam ZIP tanpa
// lewat parser XML - dua jalur, dua aturan, jangan tertukar.
//
// XSD cuma memeriksa BENTUK. Nama tipe yang tidak ada tetap lolos di sini dan baru
// ditolak Studio. Jalankan juga:  pwsh scripts/validate_xml.ps1 blurobot/sim/BlurobotSim.xml
'use strict';
const fs = require('fs');
const path = require('path');

const SIM = path.join(__dirname, '..', 'sim');
const OUT = path.join(SIM, 'BlurobotSim.xml');
const CFG = JSON.parse(fs.readFileSync(path.join(SIM, 'robot.config.json'), 'utf8'));

const SMC = 'https://www.ia.omron.com/Smc IEC61131_10_Ed1_0_SmcExt1_0_Spc1_0.xsd';
const PROGRAM = 'P_SIM_ROBOT';
const FB = ['FORWARD_KINEMATIC_V2', 'INVERSE_KINEMATIC_V2'];
const DEVICE = { modelName: 'NX102', version: '1.40' };

function esc(s) {
  return String(s == null ? '' : s)
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

const baris = f => fs.readFileSync(path.join(SIM, f), 'utf8').split('\n').filter(Boolean);
const st = f => fs.readFileSync(path.join(SIM, f), 'utf8').replace(/\r\n/g, '\n').replace(/\s+$/, '');

// --------------------------------------------------------------------- tipe
// ARRAY[0..3] OF LREAL -> InstantlyDefinedType, bukan TypeName. Ditulis sebagai
// TypeName, XSD tetap lolos (TypeName itu xsd:string apa saja) dan Studio yang
// menolak belakangan - persis kelas kegagalan yang validator ini ada untuk cegah.
function tipeXml(t, ind) {
  const m = /^ARRAY\s*\[\s*([-\d]+)\s*\.\.\s*([-\d]+)\s*\]\s*OF\s+(.+)$/i.exec(String(t).trim());
  if (!m) return ind + '<Type><TypeName>' + esc(t) + '</TypeName></Type>';
  return ind + '<Type>\n'
    + ind + '  <InstantlyDefinedType xsi:type="ArrayTypeSpec">\n'
    + ind + '    <BaseType><TypeName>' + esc(m[3].trim()) + '</TypeName></BaseType>\n'
    + ind + '    <DimensionSpec dimensionNumber="1"><IndexRange lower="' + esc(m[1])
    + '" upper="' + esc(m[2]) + '" /></DimensionSpec>\n'
    + ind + '  </InstantlyDefinedType>\n'
    + ind + '</Type>';
}

// Urutan anak Variable TERIKAT xsd:sequence, dan urutannya BUKAN urutan kolom di
// tabel Studio:  Documentation -> AddData -> Type -> InitialValue -> Address.
function varXml(v, ind, extra) {
  const b = [ind + '<Variable name="' + esc(v.nama) + '"' + (extra || '') + '>'];
  if (v.komen) b.push(ind + '  <Documentation xsi:type="SimpleText">' + esc(v.komen) + '</Documentation>');
  b.push(tipeXml(v.tipe, ind + '  '));
  if (v.awal) b.push(ind + '  <InitialValue><SimpleValue value="' + esc(v.awal) + '" /></InitialValue>');
  if (v.at) b.push(ind + '  <Address address="' + esc(v.at) + '" />');
  b.push(ind + '</Variable>');
  return b.join('\n');
}

// ------------------------------------------------------------- tabel variabel
// FB: Name, Type, Initial, Grup            (dibangkitkan gen_sim.js)
function varsFb(nama) {
  const grup = { VAR_INPUT: [], VAR_OUTPUT: [], VAR: [], VAR_EXTERNAL: [] };
  for (const l of baris(nama + '.vars.tsv')) {
    const c = l.split('\t');
    (grup[c[3]] || grup.VAR).push({ nama: c[0], tipe: c[1], awal: c[2] });
  }
  return grup;
}

// Program: Name, Type, Initial, Retain, Constant, Comment
function varsProgram() {
  return baris('ProgramVariables.tsv').map(l => {
    const c = l.split('\t');
    return { nama: c[0], tipe: c[1], awal: c[2], komen: c[5] };
  });
}

// Global: Name, Type, Initial, AT, Retain, Constant, Network Publish, Comment
function varsGlobal() {
  return baris('GlobalVariables.tsv').map(l => {
    const c = l.split('\t');
    return { nama: c[0], tipe: c[1], awal: c[2], at: c[3],
             retain: c[4] === 'True', konstan: c[5] === 'True', komen: c[7] };
  });
}

// --------------------------------------------------------------------- POU
function fbXml(nama) {
  const g = varsFb(nama);
  // Satu pencacah yang jalan terus dari input ke output - lihat kepala berkas.
  let ord = 0;
  const pin = v => varXml(v, '            ', ' orderWithinParamSet="' + (ord++) + '"');

  const b = ['      <FunctionBlock name="' + esc(nama) + '">'];
  b.push('        <Parameters>');
  b.push('          <InputVars>');
  g.VAR_INPUT.forEach(v => b.push(pin(v)));
  b.push('          </InputVars>');
  b.push('          <OutputVars>');
  g.VAR_OUTPUT.forEach(v => b.push(pin(v)));
  b.push('          </OutputVars>');
  b.push('        </Parameters>');

  // ExternalVars memakai VariableDeclPlain: nama + tipe saja, tanpa nilai awal.
  // Nilai awalnya milik variabel GLOBAL-nya, dan menaruhnya di dua tempat berarti
  // dua sumber kebenaran untuk satu angka.
  b.push('        <ExternalVars>');
  g.VAR_EXTERNAL.forEach(v => b.push(varXml({ nama: v.nama, tipe: v.tipe }, '          ')));
  b.push('        </ExternalVars>');

  b.push('        <Vars accessSpecifier="private">');
  g.VAR.forEach(v => b.push(varXml(v, '          ')));
  b.push('        </Vars>');

  b.push('        <MainBody>');
  b.push('          <BodyContent xsi:type="ST">');
  b.push('            <ST>' + esc(st(nama + '.st')) + '</ST>');
  b.push('          </BodyContent>');
  b.push('        </MainBody>');
  b.push('      </FunctionBlock>');
  return b.join('\n');
}

function programXml(globalNama) {
  const lokal = varsProgram();
  // Yang dipakai program tapi bukan variabel lokalnya = variabel global yang harus
  // dideklarasi ulang di ExternalVars. ExternalVars itu PER PROGRAM, bukan warisan:
  // simbol yang sudah ada di tabel global TETAP tidak dikenal kalau programnya tidak
  // mendeklarasikannya sendiri, dan yang lupa lolos XSD, lolos import, lalu muncul
  // sebagai variabel merah di Studio.
  const kode = st(PROGRAM + '.st').split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
  const disebut = new Set(kode.match(/[A-Za-z_][A-Za-z0-9_]*/g) || []);
  const lokalNama = new Set(lokal.map(v => v.nama));
  const ext = globalNama.filter(v => disebut.has(v.nama) && !lokalNama.has(v.nama));

  const b = ['      <Program name="' + esc(PROGRAM) + '">'];
  b.push('        <ExternalVars>');
  ext.forEach(v => b.push(varXml({ nama: v.nama, tipe: v.tipe }, '          ')));
  b.push('        </ExternalVars>');
  b.push('        <Vars accessSpecifier="private">');
  lokal.forEach(v => b.push(varXml(v, '          ')));
  b.push('        </Vars>');
  b.push('        <MainBody>');
  b.push('          <BodyContent xsi:type="ST">');
  b.push('            <ST>' + esc(st(PROGRAM + '.st')) + '</ST>');
  b.push('          </BodyContent>');
  b.push('        </MainBody>');
  b.push('      </Program>');
  return { xml: b.join('\n'), ext };
}

// Retain dan Constant itu atribut KONTAINER, bukan atribut variabel. GlobalVars
// boleh muncul berkali-kali, jadi tiap kombinasi punya kontainernya sendiri -
// itu yang dilakukan Sample.xml Omron (empat kontainer untuk empat kombinasi).
function globalXml(list) {
  const kombinasi = [[false, false], [true, false], [false, true], [true, true]];
  const out = [];
  for (const [konstan, retain] of kombinasi) {
    const isi = list.filter(v => !!v.konstan === konstan && !!v.retain === retain);
    if (!isi.length) continue;
    const atr = (konstan ? ' constant="true"' : '') + (retain ? ' retain="true"' : '');
    out.push('        <GlobalVars' + atr + '>');
    isi.forEach(v => out.push(varXml(v, '          ')));
    out.push('        </GlobalVars>');
  }
  return out.join('\n');
}

function bangun() {
  const glob = varsGlobal();
  const prog = programXml(glob);
  const b = [];
  b.push('<?xml version="1.0" encoding="utf-8"?>');
  b.push('<Project xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"');
  b.push('         xmlns:smcext="https://www.ia.omron.com/Smc"');
  b.push('         xsi:schemaLocation="https://www.ia.omron.com/Smc IEC61131_10_Ed1_0_SmcExt1_0_Spc1_0.xsd"');
  b.push('         schemaVersion="1"');
  b.push('         xmlns="www.iec.ch/public/TC65SC65BWG7TF10">');
  b.push('  <FileHeader companyName="Blurobot" productName="blurobot/tools/gen_xml.js" productVersion="1.0.0.0" />');
  b.push('  <ContentHeader name="BlurobotSim" creationDateTime="2026-01-01T00:00:00">');
  b.push('    <AddData><Data name="' + SMC + '" handleUnknown="discard">'
    + '<smcext:DeviceInfo modelName="' + DEVICE.modelName + '" version="' + DEVICE.version + '" />'
    + '</Data></AddData>');
  b.push('  </ContentHeader>');
  b.push('  <Types>');
  b.push('    <GlobalNamespace>');
  FB.forEach(n => b.push(fbXml(n)));
  b.push(prog.xml);
  b.push('    </GlobalNamespace>');
  b.push('  </Types>');
  b.push('  <Instances>');
  b.push('    <Configuration name="BlurobotSim">');
  b.push('      <Resource name="MainResource" resourceTypeName="">');
  b.push(globalXml(glob));
  b.push('      </Resource>');
  b.push('    </Configuration>');
  b.push('  </Instances>');
  b.push('</Project>');
  return { xml: b.join('\n') + '\n', glob, ext: prog.ext };
}

function main() {
  const cek = process.argv.includes('--check');
  const { xml, glob, ext } = bangun();

  if (cek) {
    if (!fs.existsSync(OUT) || fs.readFileSync(OUT, 'utf8') !== xml) {
      console.error('BASI: ' + path.basename(OUT) + '  -> jalanin: node blurobot/tools/gen_xml.js');
      process.exit(1);
    }
    console.log('OK  ' + path.basename(OUT) + ' sudah sesuai sim/');
    return;
  }
  fs.writeFileSync(OUT, xml, 'utf8');
  console.log('OK  ' + FB.length + ' FB + 1 program + ' + glob.length + ' variabel global ('
    + ext.length + ' dipakai program) -> ' + path.relative(process.cwd(), OUT));
  console.log('    validasi bentuknya:  pwsh scripts/validate_xml.ps1 '
    + path.relative(process.cwd(), OUT).replace(/\\/g, '/'));
  console.log('    TCP = ujung jari gripper (' + (CFG.tool.Y + CFG.gripper.panjang) + ' mm dari flange)');
}

main();
