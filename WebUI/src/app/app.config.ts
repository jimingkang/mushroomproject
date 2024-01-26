import { ApplicationConfig, ImportProvidersSource, importProvidersFrom } from '@angular/core';
import { provideRouter } from '@angular/router';

import { routes } from './app.routes';
import { provideHttpClient } from '@angular/common/http';


import { BrowserAnimationsModule } from '@angular/platform-browser/animations';


import { provideFirebaseApp, initializeApp } from '@angular/fire/app';
import { getAuth, provideAuth, connectAuthEmulator } from '@angular/fire/auth';
import { getFirestore, provideFirestore, connectFirestoreEmulator, enableIndexedDbPersistence } from '@angular/fire/firestore';
import { getStorage, provideStorage, connectStorageEmulator } from '@angular/fire/storage';
import { getAnalytics, provideAnalytics } from '@angular/fire/analytics';
import { getFunctions, provideFunctions, connectFunctionsEmulator} from '@angular/fire/functions';

const firebaseConfig = {
  apiKey: "AIzaSyCYPDBw1Ub8v33C2BB2PhRUIg3a57opPz4",
  authDomain: "mushroom-30b6a.firebaseapp.com",
  projectId: "mushroom-30b6a",
  storageBucket: "mushroom-30b6a.appspot.com",
  messagingSenderId: "44586902461",
  appId: "1:44586902461:web:8652274cec4148d03568d4"
};
export const appConfig: ApplicationConfig = {
  providers: [
    provideRouter(routes),provideHttpClient(),
    importProvidersFrom([BrowserAnimationsModule]),
    importProvidersFrom([
      provideFirebaseApp(() => initializeApp(firebaseConfig)),
      provideAuth(() => getAuth()),
      provideFirestore(() => getFirestore()),
      provideStorage(() => getStorage())
    
    ]
    )
  ]
    
};

